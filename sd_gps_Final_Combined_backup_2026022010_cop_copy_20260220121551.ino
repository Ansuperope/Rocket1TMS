// ---------------------------------------------------------------------
// sd_gps_combined_minopen.ino - Combined GPS, DPS/HDC, BNO055, LoRa
// Optimized: files opened once, flushed every 1s
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
const unsigned long RAM_INTERVAL = 100;     // 10Hz - read HDC, DPS sensors
const unsigned long GPS_SEND_INTERVAL = 500;// 2Hz - Send & read BNO data 
const unsigned long BNO_GET_INTERVAL = 50;  // 20Hz - read BNO

// Max size of data packets
const int BUFF_SIZE = 50;       // RAM buffer size - HDC, DPS
const int BNO_BUFF_SIZE = 100;  // BNO buff size 

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
String latestGGA="", latestRMC="";
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

  Serial7.begin(115200); // raw GPS - RMC & GGA
  Serial1.begin(115200); // LoRa Gun
  Serial2.begin(115200); // LoRa Main
  Serial8.begin(9600);   // GPS serial - BNO

  delay(200);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  Serial1.println("AT+ADDRESS=3"); Serial1.println("AT+NETWORKID=19");
  Serial2.println("AT+ADDRESS=4"); Serial2.println("AT+NETWORKID=18");

  if(!dps.begin_I2C()) while(1);
  if(!hdc.begin(0x44,&Wire)) while(1);
  if(!bno.begin()) while(1);

  if(!SD.begin(BUILTIN_SDCARD)) while(1);

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
}

// -------------------- LOOP --------------------
void loop() {
  unsigned long now = millis();

  readGPS();

  // DPS/HDC sampling
  if(now - lastDPSHDC >= RAM_INTERVAL){
    lastDPSHDC = now;
    ramBuffer[writeIndex].time = now;
    readDPS(writeIndex);
    readHDC(writeIndex);
    writeIndex++;
    if(writeIndex>=BUFF_SIZE) writeIndex=0;
  }

  // BNO055 sampling 100Hz
  if(now - lastBNO >= BNO_GET_INTERVAL){
    lastBNO = now;

    sensors_event_t raw;
    bno.getEvent(&raw, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    float accX = raw.acceleration.x - gravity.x();
    float accY = raw.acceleration.y - gravity.y();
    float accZ = raw.acceleration.z - gravity.z();
    accX_smooth = alpha*accX + (1-alpha)*accX_smooth;
    accY_smooth = alpha*accY + (1-alpha)*accY_smooth;
    accZ_smooth = alpha*accZ + (1-alpha)*accZ_smooth;

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

  // SD write every 1s - saves ALL data
  static unsigned long lastSD = 0;
  if(now - lastSD >= SD_INTERVAL){
    lastSD = now;

    // DPS/HDC
    for(int i=0;i<writeIndex;i++){
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
    for(int i=0;i<bnoIndex;i++){
      //fileBNO.print("\"");
      fileBNO.print(bnoBuffer[i].time); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].velocity); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].latitude,6); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].longitude,6); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].altitude); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].accX); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].accY); fileBNO.print(",");
      fileBNO.print(bnoBuffer[i].accZ);
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
  }

  // LoRa send every 
  if(now - lastLoRa >= GPS_SEND_INTERVAL){
    lastLoRa = now;

    if(latestGGA.length()>0) loraSend(LORA_MAIN, ADDR_MAIN, latestGGA);
    if(latestRMC.length()>0) loraSend(LORA_MAIN, ADDR_MAIN, latestRMC);
    
    char telemetry[150];
    snprintf(telemetry,sizeof(telemetry),
      "%02d:%02d:%02d,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f",
      lastHour,lastMinute,lastSecond,
      lastVelocity,lastLatitude,lastLongitude,lastAltitude,
      accX_smooth,accY_smooth,accZ_smooth
    );
    loraSend(LORA_GUN, ADDR_GUN, telemetry);
  }
}

// -------------------- FUNCTIONS --------------------
void readDPS(int i){
  sensors_event_t event;
  dps.getTemperatureSensor()->getEvent(&event);
  ramBuffer[i].tempDPS=event.temperature;

  dps.getPressureSensor()->getEvent(&event);
  ramBuffer[i].pressureDPS=event.pressure;

  ramBuffer[i].altitudeDPS=dps.readAltitude(SEA_LEVEL);
}

void readHDC(int i){
  double t,rh;
  if(hdc.readTemperatureHumidityOnDemand(t,rh,TRIGGERMODE_LP0)){
    ramBuffer[i].tempHDC=(float)t;
    ramBuffer[i].humidityHDC=(float)rh;
  }
}

void createCSV(const char* f,const char* h){
  if(!SD.exists(f)){
    File ff=SD.open(f,FILE_WRITE);
    if(ff){ff.println(h); ff.close();}
  }
}

void readGPS(){
  while(Serial7.available()){
    char c = Serial7.read();
    if(c=='$'){ nmeaIndex=0; collectingSentence=true; nmeaLine[nmeaIndex++]=c; continue;}
    if(!collectingSentence) continue;
    if(nmeaIndex>=GPS_LINE_MAX-1){ collectingSentence=false; nmeaIndex=0; return;}
    nmeaLine[nmeaIndex++] = c;
    if(c=='\n'){ nmeaLine[nmeaIndex]='\0'; collectingSentence=false; processNMEASentence(nmeaLine);}
  }
}

void processNMEASentence(const char* s){
  String str = s;
  str.replace("\r",""); str.replace("\n","");

  // Write to GPS file
  if(fileGPS){ 
    fileGPS.print(millis());
    fileGPS.print(",\"");
    fileGPS.print(str);
    fileGPS.println("\"");
  }

  if(!nmeaChecksumOK(s)) return;
  if(str.startsWith("$GPGGA")||str.startsWith("$GNGGA")) latestGGA=str;
  else if(str.startsWith("$GPRMC")||str.startsWith("$GNRMC")) latestRMC=str;

  // Fix type mismatch here
  char temp[GPS_LINE_MAX];
  str.toCharArray(temp, GPS_LINE_MAX);
  if(GPS.parse(temp)){ // now it works
      if(GPS.fix){
          lastHour = GPS.hour; 
          lastMinute = GPS.minute;
          lastSecond = GPS.seconds;
          lastVelocity = GPS.speed * KNOTS_TO_FPS;
          lastAltitude = GPS.altitude * METERS_TO_FEET;
          lastLatitude = GPS.latitudeDegrees;
          lastLongitude = GPS.longitudeDegrees;
      }
  }
}

bool nmeaChecksumOK(const char* s){
  if(s[0]!='$') return false;
  const char* star=strchr(s,'*'); if(!star) return false;
  uint8_t sum=0; for(const char* p=s+1;p<star;p++) sum^=*p;
  if(*(star+1)=='\0'||*(star+2)=='\0') return false;
  auto hex=[](char c){if(c>='0'&&c<='9')return c-'0'; if(c>='A'&&c<='F')return 10+(c-'A'); if(c>='a'&&c<='f')return 10+(c-'a'); return -1;};
  int v1=hex(*(star+1)),v2=hex(*(star+2)); if(v1<0||v2<0) return false;
  return sum==((v1<<4)|v2);
}

void loraSend(HardwareSerial &port,uint16_t addr,const String &payload){
  String clean=payload;
  clean.replace("\r","");
  clean.replace("\n","");
  port.print("AT+SEND=");
  port.print(addr); port.print(",");
  port.print(clean.length());
  port.print(",");
  port.println(clean);
}
