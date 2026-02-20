// ---------------------------------------------------------------------
// sd_gps_combined.ino - lora2, Main GPS, going to reciever
// ---------------------------------------------------------------------
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_HDC302x.h>

#define GPS_LINE_MAX 120
#define LORA_GUN   Serial1   // LoRa on Serial1
#define LORA_MAIN  Serial2   // LoRa on Serial2

// Destination addresses 
const uint16_t ADDR_GUN  = 2;  // gun receiver address on Network 19
const uint16_t ADDR_MAIN = 1;  // main receiver address on Network 18 (set to whatever the main receiver uses) 
 

// ------------------------- CONSTANTS --------------------------
const unsigned long SD_INTERVAL  = 1000;  // Write to SD every 1 s
const unsigned long RAM_INTERVAL = 200;   // RAM buffer interval
const unsigned long SEND_INTERVAL_MS = 1000; // 1 Hz transmit

const int BUFF_SIZE = 50;                 // RAM buffer size
const float SEA_LEVEL = 1013.25;          // Standard atmosphere
const int GPS_DATA_LENGTH = 120;

// ------------------------- GLOBAL VARIABLES --------------------------
Adafruit_DPS310 dps;
Adafruit_HDC302x hdc = Adafruit_HDC302x();

unsigned long lastRamTime = 0;
unsigned long lastSDTime  = 0;

File fileGPS;

// ------------------------- DATA PACKET --------------------------
struct DataPacket {
    unsigned long time;                   // Timestamp
    float tempDPS;
    float tempHDC;
    float pressureDPS;
    float humidityHDC;
    float altitudeDPS;
    char gpsGGA[GPS_DATA_LENGTH];   // GPS GGA sentence
    char gpsRMC[GPS_DATA_LENGTH];   // GPS RMC sentence
};

DataPacket ramBuffer[BUFF_SIZE];
int writeIndex = 0;

// GPS variables
char nmeaLine[GPS_LINE_MAX];
String latestGGA = "";
String latestRMC = "";


uint8_t nmeaIndex = 0;
bool collectingSentence = false;

// ---------------- LORA SETTINGS ----------------
unsigned long lastSendTime = 0;
unsigned long lastAnySendMs = 0;
bool toggle = false;

// Parsed GPS values
float gpsLat = 0.0;
float gpsLon = 0.0;
int gpsFix = 0;
int gpsSats = 0;

// ------------------------- SETUP --------------------------
void setup() {
  // LoRa 
  Serial.begin(115200);
  Wire.begin();


  // GPS (Teensy Serial7)
  Serial7.begin(115200);
  delay(200);


  // LoRa #Gun (Serial1) - Network 19
  Serial1.begin(115200);
  delay(200);
  Serial1.println("AT");
  delay(200);
  Serial1.println("AT+NETWORKID=19");
  delay(200);
  Serial1.println("AT+ADDRESS=3");   // rocket is address 1
  delay(200);

  // LoRa #Main (Serial2) - Network 18
  Serial2.begin(115200);
  delay(200);
  Serial2.println("AT");
  delay(200);
  Serial2.println("AT+NETWORKID=18");
  delay(200);
  Serial2.println("AT+ADDRESS=4");   // rocket is address 1
  delay(200);


  Serial.println("Initializing SD card...");

  // SD 
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Failed to find SD Card!");
    while (1);
  }

  // DPS
  if (!dps.begin_I2C()) {
    Serial.println("Failed to find DPS310 chip!");
    while (1);
  }

  // HDC
  if (!hdc.begin(0x44, &Wire)) {
    Serial.println("Failed to find HDC302 chip");
    while (1);
  }

  // GPS
  fileGPS = SD.open("gps.csv", FILE_WRITE);
  if (!fileGPS) {
    Serial.println("Failed to create gps.csv!");
  }

  // Create CSV headers if not exists
  createCSV("temp.csv",       "Time,Temp DPS,Temp HDC");
  createCSV("pressure.csv",   "Time,Pressure");
  createCSV("humid.csv",      "Time,Humidity");
  createCSV("altitude.csv",   "Time,Altitude");

  Serial.println("All hardware initialized :)");
  delay(500);
}

// ------------------------- LOOP --------------------------
void loop() {
  // --- GPS reading ---
  readGPS();

  // --- RAM buffer - all sensors except GPS ---
  if (millis() - lastRamTime >= RAM_INTERVAL && writeIndex < BUFF_SIZE) {
    lastRamTime = millis();

    ramBuffer[writeIndex].time = millis();
    readDPS(writeIndex);

    readHDC(writeIndex);

    writeIndex++;
  }

  // --- SD write - All sensors except GPS ---
  if (millis() - lastSDTime >= SD_INTERVAL || writeIndex >= BUFF_SIZE) {
    lastSDTime = millis();
    writeSD();
    writeIndex = 0;
  }

  // ---------------- LORA TRANSMIT (MAIN ONLY for now) ----------------
  if (millis() - lastAnySendMs >= SEND_INTERVAL_MS) {

    // Send GGA then RMC alternating
  if (!toggle && latestGGA.length() > 0) {
    // MAIN (Network 18 on Serial2)
    loraSend(LORA_MAIN, ADDR_MAIN, latestGGA);
      
    // GUN (Network 19 on Serial1)
    loraSend(LORA_GUN,  ADDR_GUN,  latestGGA);


      toggle = true;
      lastAnySendMs = millis();

    } else if (toggle && latestRMC.length() > 0) {
      loraSend(LORA_MAIN, ADDR_MAIN, latestRMC);
      loraSend(LORA_GUN,  ADDR_GUN,  latestRMC);
      
      toggle = false;
      lastAnySendMs = millis();
    }
  }

    // Optional: read LoRa responses for debug
    while (Serial2.available()) Serial.write(Serial2.read());
    while (Serial1.available()) Serial.write(Serial1.read());
    

    // --- GPS Write to SD ---
    static unsigned long lastFlush = 0;
    if (millis() - lastFlush > 1000) {
    if (fileGPS) fileGPS.flush();
    lastFlush = millis();
  }

}

// ------------------------- GPS READ --------------------------
void parseGGA(const String &s) {
  int field = 0;
  int lastIndex = 0;

  float rawLat = 0.0;
  float rawLon = 0.0;
  char latDir = 'N';
  char lonDir = 'E';

  for (int i = 0; i < (int)s.length(); i++) {
    if (s[i] == ',' || s[i] == '*') {
      String token = s.substring(lastIndex, i);
      lastIndex = i + 1;

      switch (field) {
        case 2: rawLat = token.toFloat(); break;
        case 3: latDir = token[0]; break;
        case 4: rawLon = token.toFloat(); break;
        case 5: lonDir = token[0]; break;
        case 6: gpsFix = token.toInt(); break;
        case 7: gpsSats = token.toInt(); break;
      }

      field++;
    }
  }

  int latDeg = (int)(rawLat / 100);
  float latMin = rawLat - (latDeg * 100);
  gpsLat = latDeg + (latMin / 60.0);

  int lonDeg = (int)(rawLon / 100);
  float lonMin = rawLon - (lonDeg * 100);
  gpsLon = lonDeg + (lonMin / 60.0);

  if (latDir == 'S') gpsLat = -gpsLat;
  if (lonDir == 'W') gpsLon = -gpsLon;
}

// ------------------------- GPS READ --------------------------
void parseRMC(const String &s) {
  int field = 0;
  int lastIndex = 0;

  float rawLat = 0.0;
  float rawLon = 0.0;
  char latDir = 'N';
  char lonDir = 'E';

  for (int i = 0; i < (int)s.length(); i++) {
    if (s[i] == ',' || s[i] == '*') {
      String token = s.substring(lastIndex, i);
      lastIndex = i + 1;

      switch (field) {
        case 2: rawLat = token.toFloat(); break;
        case 3: latDir = token[0]; break;
        case 4: rawLon = token.toFloat(); break;
        case 5: lonDir = token[0]; break;
        case 6: gpsFix = token.toInt(); break;
        case 7: gpsSats = token.toInt(); break;
      }

      field++;
    }
  }

  int latDeg = (int)(rawLat / 100);
  float latMin = rawLat - (latDeg * 100);
  gpsLat = latDeg + (latMin / 60.0);

  int lonDeg = (int)(rawLon / 100);
  float lonMin = rawLon - (lonDeg * 100);
  gpsLon = lonDeg + (lonMin / 60.0);

  if (latDir == 'S') gpsLat = -gpsLat;
  if (lonDir == 'W') gpsLon = -gpsLon;
}

void readGPS() {
  while (Serial7.available()) {
    char c = Serial7.read();
    // Serial.write(c); - for debugging, see if sentences are made

    if (c == '$') {
      nmeaIndex = 0;
      collectingSentence = true;
      nmeaLine[nmeaIndex++] = c;
      continue;
    }

    if (!collectingSentence) continue;

    if (nmeaIndex >= GPS_LINE_MAX - 1) {
      collectingSentence = false;
      nmeaIndex = 0;
      return;
    }

    nmeaLine[nmeaIndex++] = c;

    if (c == '\n') {

      nmeaLine[nmeaIndex] = '\0';
      collectingSentence = false;

      processNMEASentence(nmeaLine);
    }
  }
}

void processNMEASentence(const char* sentence) {
    // Strip newline/carriage return
    String s = String(sentence);
    s.replace("\r", "");
    s.replace("\n", "");

    // Optional: save to SD
    if (fileGPS) {
        fileGPS.print(millis());
        fileGPS.print(",\"");
        fileGPS.print(s);
        fileGPS.println("\"");
    }

    if (!nmeaChecksumOK(sentence)) return;

    if (s.startsWith("$GPGGA") || s.startsWith("$GNGGA")) {
        latestGGA = s;
        Serial.println(latestGGA);
    } else if (s.startsWith("$GPRMC") || s.startsWith("$GNRMC")) {
        latestRMC = s;
        Serial.println(latestRMC);
    }
}

bool nmeaChecksumOK(const char* s) {
    if (s[0] != '$') return false;

    const char* star = strchr(s, '*');
    if (!star) return false;

    uint8_t sum = 0;
    for (const char* p = s + 1; p < star; p++) {
        sum ^= (uint8_t)(*p);
    }

    if (*(star + 1) == '\0' || *(star + 2) == '\0') return false;

    auto hexVal = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
        if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
        return -1;
    };

    int v1 = hexVal(*(star + 1));
    int v2 = hexVal(*(star + 2));
    if (v1 < 0 || v2 < 0) return false;

    return sum == ((v1 << 4) | v2);
}


void loraSend(HardwareSerial &port, uint16_t destination, const String &payload) {
  String clean = payload;
  clean.replace("\r", "");
  clean.replace("\n", "");

  // Confirm you are actually transmitting on BOTH radios
  if (&port == &Serial1) Serial.println("Sending on Serial1 (GUN LoRa)");
  if (&port == &Serial2) Serial.println("Sending on Serial2 (MAIN LoRa)");


  port.print("AT+SEND=");
  port.print(destination);
  port.print(",");
  port.print(clean.length());
  port.print(",");
  port.println(clean);

  Serial.print("TX on port ->");
  Serial.println(clean);
}

// void loraSend(uint16_t destination, const String &payload) {
//     String clean = payload;
//     clean.replace("\r", "");
//     clean.replace("\n", "");

//     Serial2.print("AT+SEND=");
//     Serial2.print(destination);
//     Serial2.print(",");
//     Serial2.print(clean.length());
//     Serial2.print(",");
//     Serial2.println(clean);

//     Serial.print("Debugging TX: ");
//     Serial.println(clean);
// }



// ------------------------- DPS READ --------------------------
void readDPS(const int index) {
  sensors_event_t event;

  dps.getTemperatureSensor()->getEvent(&event);
  ramBuffer[index].tempDPS = event.temperature;

  dps.getPressureSensor()->getEvent(&event);
  ramBuffer[index].pressureDPS = event.pressure;

  ramBuffer[index].altitudeDPS = dps.readAltitude(SEA_LEVEL);
}

// ------------------------- HDC READ --------------------------
void readHDC(int index) {
    double temp, rh;  // must be double for function

    if(hdc.readTemperatureHumidityOnDemand(temp, rh, TRIGGERMODE_LP0)) {
        ramBuffer[index].tempHDC = (float)temp;
        ramBuffer[index].humidityHDC = (float)rh;
    }
    // ERROR 
    else {
        Serial.println("Bruh. Can't read HDC data :(");
    }
}

// ------------------------- SD WRITE --------------------------
void writeSD() {
  File fTemp     = SD.open("temp.csv", FILE_WRITE);
  File fPressure = SD.open("pressure.csv", FILE_WRITE);
  File fHumid    = SD.open("humid.csv", FILE_WRITE);
  File fAltitude = SD.open("altitude.csv", FILE_WRITE);

  if (!fTemp || !fPressure || !fHumid || !fAltitude) {
    Serial.println("SD open failed!");
    return;
  }

  for (int i = 0; i < writeIndex; i++) {

    // TEMP 
    fTemp.print(ramBuffer[i].time); fTemp.print(",");
    fTemp.print(ramBuffer[i].tempDPS); fTemp.print(","); fTemp.println(ramBuffer[i].tempHDC);

    // PRESSURE
    fPressure.print(ramBuffer[i].time); fPressure.print(",");
    fPressure.println(ramBuffer[i].pressureDPS);

    // HUMID
    fHumid.print(ramBuffer[i].time); fHumid.print(",");
    fHumid.println(ramBuffer[i].humidityHDC);

    // ALTITUDE
    fAltitude.print(ramBuffer[i].time); fAltitude.print(",");
    fAltitude.println(ramBuffer[i].altitudeDPS);
  }

  fTemp.close(); fPressure.close(); fHumid.close(); fAltitude.close();
}

// ------------------------- CREATE CSV --------------------------
void createCSV(const char* filename, const char* header) {
  if (!SD.exists(filename)) {
    File f = SD.open(filename, FILE_WRITE);
    if (f) { f.println(header); f.close(); }
  }
}
