// ---------------------------------------------------------------------
// sd_gps_combined_minopen.ino - Combined GPS, DPS/HDC, BNO055, LoRa
// Optimized: files opened once, flushed every 1s
//
// Network 18 Lora 2 / Main GPS -> Main Reciever
//  Reciever: 
//    1. RMC nmea sentences
//    2. GGA nmea sentences
//  SD:  
//    1. ALL nmea sentences
//    2. temp, accel, imu, pressure, all sensors
// Network 19 Lora 1 / Second GPS -> GPS
//  Reciever:
//    1. 
// ---------------------------------------------------------------------
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_HDC302x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>


// ----- ALTER ANY OF THESE ----
// Rates to run things
const unsigned long SD_INTERVAL  = 1000;    // 1Hz - save all data every second
const unsigned long RAM_INTERVAL = 200;     // 5Hz - read HDC, DPS sensors
const unsigned long GPS_SEND_INTERVAL = 500;// 2Hz - Send & read RMC and GGA data 
const unsigned long BNO_GET_INTERVAL = 500;  // 2Hz - read BNO

// Max size of data packets
const int BUFF_SIZE = 50;       // RAM buffer size - HDC, DPS
const int BNO_BUFF_SIZE = 120;  // BNO buff size 

// for calculations
const float SEA_LEVEL = 1013.25; // set start at sea level
#define KNOTS_TO_FPS 1.68781
#define METERS_TO_FEET 3.28084
// ----- END ALTERING - DONT TOUCH BELOW UNLESS YOU KNOW WHAT YOURE DOING ----

// -------------------- HARDWARE --------------------
Adafruit_DPS310 dps;
Adafruit_HDC302x hdc;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_GPS GPS(&Serial8);

// -------------------- CONSTANTS --------------------
#define GPS_LINE_MAX 120
#define LORA_GUN   Serial1
#define LORA_MAIN  Serial2
const uint16_t ADDR_GUN  = 2;
const uint16_t ADDR_MAIN = 1;

// -------------------- GLOBALS --------------------
// RAM buffers
struct DataPacket { 
  unsigned long time; 
  float tempDPS,tempHDC,pressureDPS,humidityHDC,altitudeDPS; 
};

DataPacket ramBuffer[BUFF_SIZE];
int writeIndex = 0;

struct BnoPacket { 
  unsigned long time;
  float velocity,latitude,longitude,altitude,accX,accY,accZ; 
};

BnoPacket bnoBuffer[BNO_BUFF_SIZE];
int bnoIndex = 0;

// GPS
char nmeaLine[GPS_LINE_MAX];
char latestGGA[GPS_LINE_MAX] = {0};
char latestRMC[GPS_LINE_MAX] = {0};
uint8_t nmeaIndex=0; bool collectingSentence=false;
int lastHour=0,lastMinute=0,lastSecond=0;
float lastVelocity=0,lastAltitude=0,lastLatitude=0,lastLongitude=0;

// BNO low-pass filter
float alpha=0.1, accX_smooth=0, accY_smooth=0, accZ_smooth=0;

// Timing
unsigned long lastDPSHDC=0, lastBNO=0, lastLoRa=0;


// -------------------- FILES --------------------
File fileGPS, fileTemp, filePressure, fileHumid, fileAltitude, fileBNO;

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial1.begin(115200);    // LoRa 1, Gun
  Serial2.begin(115200);    // LoRa 2, Main
  // Serial5.begin(115200); // Adafruit GPS
  Serial7.begin(115200);    // Main GPS - RMC & GGA
  Serial8.begin(115200);    // GPS serial - BNO

  delay(200);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  Serial1.println("AT+ADDRESS=3"); Serial1.println("AT+NETWORKID=19");
  Serial2.println("AT+ADDRESS=4"); Serial2.println("AT+NETWORKID=18");

  if(!dps.begin_I2C()) sendError("DPS", "not detected");;
  if(!hdc.begin(0x44,&Wire)) sendError("HDC", "not detected");;
  if(!bno.begin()) sendError("BNO", "not detected");;

  if(!SD.begin(BUILTIN_SDCARD)) sendError("SD", "not detected");;

  createCSV("temp.csv","Time,TempDPS,TempHDC");
  createCSV("pressure.csv","Time,Pressure");
  createCSV("humid.csv","Time,Humidity");
  createCSV("altitude.csv","Time,Altitude");
  createCSV("gps.csv","Time,NMEA");
  createCSV("bno.csv","Time,Velocity,Lat,Long,Alt,AccX,AccY,AccZ");

  // Open files ONCE
  fileGPS = SD.open("gps.csv", FILE_WRITE);
  fileTemp = SD.open("temp.csv", FILE_WRITE);
  filePressure = SD.open("pressure.csv", FILE_WRITE);
  fileHumid = SD.open("humid.csv", FILE_WRITE);
  fileAltitude = SD.open("altitude.csv", FILE_WRITE);
  fileBNO = SD.open("bno.csv", FILE_WRITE);

  // Set Clock
  // setSyncProvider(Teensy3Clock.get);
}

// -------------------- LOOP --------------------
void loop() {
  unsigned long now = millis();

  readGPS();      // main GPS - RMC and GGA
  readSecondGPS(); // Serial8 GPS - IMU (vel, long, lat, alt, accel, etc)

  // ==============================
  // DPS/HDC sampling - 10HZ
  // ==============================
  if(now - lastDPSHDC >= RAM_INTERVAL){
    lastDPSHDC = now;
    ramBuffer[writeIndex].time = now;
    readDPS(writeIndex);
    readHDC(writeIndex);
    writeIndex++;
    if(writeIndex >= BUFF_SIZE) writeIndex=0;
  } // END DPS / HDC

  // ==============================
  // TRANSMIT TO MAIN RECIEVER
  // ==============================
  if(now - lastLoRa >= GPS_SEND_INTERVAL){
    lastLoRa = now;

    if(latestGGA[0] != '\0') loraSend(LORA_MAIN, ADDR_MAIN, latestGGA);
    if(latestRMC[0] != '\0') loraSend(LORA_MAIN, ADDR_MAIN, latestRMC);
  } // END transmit to main

  // ==============================
  // TRANSMIT TO GUN
  // ==============================
  if (now - lastBNO >= BNO_GET_INTERVAL) {
    lastBNO = now;
    readBNO(now);

    char telemetry[150];
    snprintf(telemetry,sizeof(telemetry),
      "%02d:%02d:%02d,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f",
      lastHour,lastMinute,lastSecond,
      lastVelocity,lastLatitude,lastLongitude,lastAltitude,
      accX_smooth,accY_smooth,accZ_smooth
    );

    loraSend(LORA_GUN, ADDR_GUN, telemetry);
    Serial.print("To Gun: ");
    Serial.println(telemetry);
  } // END tranmit to gun

  
  // ==============================
  // SAVE TO SD - 1 PER SEC
  // ==============================
  static unsigned long lastSD = 0;
  if(now - lastSD >= SD_INTERVAL){
    lastSD = now;

    // DPS/HDC
    for(int i = 0; i < writeIndex; i++){
      fileTemp.print(ramBuffer[i].time); fileTemp.print(",");
      fileTemp.print(ramBuffer[i].tempDPS); fileTemp.print(",");
      fileTemp.println(ramBuffer[i].tempHDC);

      filePressure.print(ramBuffer[i].time); filePressure.print(",");
      filePressure.println(ramBuffer[i].pressureDPS);

      fileHumid.print(ramBuffer[i].time); fileHumid.print(",");
      fileHumid.println(ramBuffer[i].humidityHDC);

      fileAltitude.print(ramBuffer[i].time); fileAltitude.print(",");
      fileAltitude.println(ramBuffer[i].altitudeDPS);
    }
    writeIndex = 0;

    // BNO buffer
    for(int i = 0; i < bnoIndex; i++){
      //fileBNO.print("\"");
      fileBNO.print(bnoBuffer[i].time); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].velocity); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].latitude,6); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].longitude,6); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].altitude); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].accX); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].accY); fileBNO.print(",");
      fileBNO.println(bnoBuffer[i].accZ);
      //fileBNO.println("\"");
    }
    bnoIndex = 0;

    // flush all files
    // temp closes and opens files to try to lower chances of corruption
    fileTemp.flush();
    filePressure.flush();
    fileHumid.flush();
    fileAltitude.flush();
    fileBNO.flush();
    fileGPS.flush();
  } // END write to SD

} // END loop()

// -------------------- FUNCTIONS --------------------
// ==============================
// ERROR MESSAGE - HARDWARE ISSUE
// ==============================
void sendError(const char* component, const char* message) {
    // PRINT TO SERIAL MONITOR
    Serial.print("Error with chip: ");
    Serial.println(component);

    char errorPacket[80];

    snprintf(errorPacket, sizeof(errorPacket),
             "ERROR! %s, %s",
             message, component);

    Serial.println(errorPacket);

    loraSend(LORA_MAIN, ADDR_MAIN, errorPacket); // send to receiver address 2
}

// ==============================
// READ DPS
// ==============================
void readDPS(int i){
  sensors_event_t event;
  dps.getTemperatureSensor()->getEvent(&event);
  ramBuffer[i].tempDPS=event.temperature;

  dps.getPressureSensor()->getEvent(&event);
  ramBuffer[i].pressureDPS=event.pressure;

  ramBuffer[i].altitudeDPS=dps.readAltitude(SEA_LEVEL);
} // END readDPS

void readHDC(int i){
  double t,rh;
  if(hdc.readTemperatureHumidityOnDemand(t,rh,TRIGGERMODE_LP0)){
    ramBuffer[i].tempHDC=(float)t;
    ramBuffer[i].humidityHDC=(float)rh;
  }
} // END readHDC

// ==============================
// CREATE CSV FILE
// ==============================
void createCSV(const char* f,const char* h){
  if(!SD.exists(f)){
    File ff=SD.open(f,FILE_WRITE);
    if(ff){ff.println(h); ff.close();}
  }
} // END createCSV

// ==============================
// GET ALL GPS DATA
// ==============================
void readGPS(){
  while(Serial7.available()){
    char c = Serial7.read();
    if(c=='$') { 
      nmeaIndex=0; 
      collectingSentence=true; 
      nmeaLine[nmeaIndex++]=c; 
      continue;
    }
    if(!collectingSentence) continue;
    if(nmeaIndex>=GPS_LINE_MAX-1){ 
      collectingSentence=false; 
      nmeaIndex=0; 
      return;
    }
    nmeaLine[nmeaIndex++] = c;
    if(c=='\n'){ 
      nmeaLine[nmeaIndex]='\0'; 
      collectingSentence=false; 
      processNMEASentence(nmeaLine);
    }
  }
} // END readGPS()

// ==============================
// CREATE NMEA SENTENCE + WRITE TO GPS
// ==============================
void processNMEASentence(const char* s){

  // Write raw NMEA to SD (no String used)
  if(fileGPS){
    fileGPS.print(millis());
    fileGPS.print(",\"");
    fileGPS.print(s);        // s already null terminated
    fileGPS.println("\"");
  }

  // Validate checksum
  if(!nmeaChecksumOK(s)) return;

  // Detect GGA / RMC using C string comparison
  if(strncmp(s, "$GPGGA", 6) == 0 || strncmp(s, "$GNGGA", 6) == 0){
    snprintf(latestGGA, GPS_LINE_MAX, "%s", s);
  }
  else if(strncmp(s, "$GPRMC", 6) == 0 || strncmp(s, "$GNRMC", 6) == 0){
    snprintf(latestRMC, GPS_LINE_MAX, "%s", s);
  }

  // Parse requires NON-CONST char*
  char temp[GPS_LINE_MAX];
  strncpy(temp, s, GPS_LINE_MAX);
  temp[GPS_LINE_MAX - 1] = '\0';

} // END processNMEASentence

// Read BNO
void readBNO(unsigned long now) {

  sensors_event_t raw;
  bno.getEvent(&raw, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  float accX = raw.acceleration.x - gravity.x();
  float accY = raw.acceleration.y - gravity.y();
  float accZ = raw.acceleration.z - gravity.z();

  accX_smooth = alpha*accX + (1-alpha)*accX_smooth;
  accY_smooth = alpha*accY + (1-alpha)*accY_smooth;
  accZ_smooth = alpha*accZ + (1-alpha)*accZ_smooth;

  float threshold = 0.05;
  if (fabs(accX_smooth) < threshold) accX_smooth = 0;
  if (fabs(accY_smooth) < threshold) accY_smooth = 0;
  if (fabs(accZ_smooth) < threshold) accZ_smooth = 0;

  if(bnoIndex < BNO_BUFF_SIZE){
    bnoBuffer[bnoIndex].time = now;
    bnoBuffer[bnoIndex].velocity = lastVelocity;
    bnoBuffer[bnoIndex].latitude = lastLatitude;
    bnoBuffer[bnoIndex].longitude = lastLongitude;
    bnoBuffer[bnoIndex].altitude = lastAltitude;
    bnoBuffer[bnoIndex].accX = accX_smooth;
    bnoBuffer[bnoIndex].accY = accY_smooth;
    bnoBuffer[bnoIndex].accZ = accZ_smooth;
    bnoIndex++;
  }
}

void readSecondGPS() {

  GPS.read();

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;

    if (GPS.fix) {

      lastHour = GPS.hour - 8;
      if (lastHour < 0) lastHour += 24;

      lastMinute   = GPS.minute;
      lastSecond   = GPS.seconds;

      lastVelocity = GPS.speed * KNOTS_TO_FPS;
      lastAltitude = GPS.altitude * METERS_TO_FEET;
      lastLatitude = GPS.latitudeDegrees;
      lastLongitude= GPS.longitudeDegrees;
    }
  }
}

// ==============================
// CHECK NMEA CHECK SUM - NUM AFTER *
// ==============================
bool nmeaChecksumOK(const char* s){
  if(s[0]!='$') return false;
  const char* star=strchr(s,'*');
  if(!star) return false;
  uint8_t sum=0; 
  for(const char* p=s+1;p<star;p++) sum^=*p;
  if(*(star+1)=='\0'||*(star+2)=='\0') return false;
  auto hex=[](char c){
    if(c>='0'&&c<='9')return c-'0'; 
    if(c>='A'&&c<='F')return 10+(c-'A'); 
    if(c>='a'&&c<='f')return 10+(c-'a'); 
    return -1;
  };
  int v1=hex(*(star+1)),v2=hex(*(star+2)); 
  if(v1<0||v2<0) return false;
  return sum==((v1<<4)|v2);
} // END numeaChecksumOk

// ==============================
// TEMPLATE FUNCTION - SEND TO RECEIVERS
// ==============================
void loraSend(HardwareSerial &port, uint16_t addr, const char* payload){
  port.print("AT+SEND=");
  port.print(addr);
  port.print(",");
  port.print(strlen(payload));
  port.print(",");
  port.println(payload);

  Serial.print("To Reciever: ");
  Serial.println(payload);
} // END loraSend
