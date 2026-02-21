// ---------------------------------------------------------------------
// sd.ino
// ---------------------------------------------------------------------
//  ABOUT
// This file will do the following:
//  1. Collect data:
//      a. DPS - 5 Hz
//      b. HDC - 5 Hz
//      c. IMU - 20 Hz (not done yet)
//      d. GPS - 5 Hz (not done yet)
//
//  2. After 1 second passes it will save data to the SD card:
//      a. temp.csv     - saves temp data
//      b. all.csv      - saves all data in one file
//
// Notes:
//  * Hz (hertz) = 1 reading per second
// ---------------------------------------------------------------------
//  PREREQUISITES
// 
// ---------------------------------------------------------------------
//  FUNCTIONS
// millis() - https://docs.arduino.cc/language-reference/en/functions/time/millis/
// ---------------------------------------------------------------------

// ----------------------------- LIBRARIES -----------------------------
#include <SPI.h>                // For SPI connections
#include <SD.h>                 // SD library for SD.begin
#include <Wire.h>               // Used for hardware connections
#include <Adafruit_DPS310.h>    // library for DPS310
#include <Adafruit_HDC302x.h>   // library for HDC302


// ------------------------- GLOBAL CONSTANTS --------------------------
// Note: rate (Hz) = 1000 (seconds) / interval
//       interval  = 1000 (seconds) / rate you want (Hz)
const unsigned long SD_INTERVAL = 1000;     // Writing to SD every second
const unsigned long RAM_INTERVAL = 200;     // RAM Buffer rate

const int BUFF_SIZE = 50; // max number samples / packets to save / send 

const float SEA_LEVEL = 1013.25;  // standart atmosphere

// ------------------------- GLOBAL VARIABLES --------------------------
// FOR FILES
// File Serial;          // time
// File Serial;          // temperture
// File Serial;
// File Serial;
// File Serial;

// DPS COMPONENT VARAIBLES
Adafruit_DPS310 dps;          // object variable so we can use library

// HDC COMPONENT VARAIBLES
Adafruit_HDC302x hdc = Adafruit_HDC302x();  // object variable so we can
                                            // use library

// KEEPING TRACK OF TIME - FOR INTERVAL / RATE
unsigned long lastRamTime = 0;  // time we last input to RAM
unsigned long lastSDTime = 0;   // time we last saved to SD

// PACKET - STRUCTURE OF DATA - USED TO SAVE AND STORE DATA
struct DataPacket {
    uint32_t time;          // time data logged / saved

    float tempDPS;          // temperature from DPS
    float tempHDC;          // temperature from HDC

    float pressureDPS;

    float humidityHDC;      // humidity form HDC

    float altitudeDPS;      // altitude from DPS
}; // END DataPacket

// RAM BUFFER - QUEUE DATA STRUCTURE - DATA FIRST IN FIRST OUT
DataPacket ramBuffer[BUFF_SIZE];    // ram buffer, data we collected
int writeIndex = 0;                 // keeps track of how many we write

// ---------------------------------------------------------------------
// Code that will run once - checking connections
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(9600);   // baur rate
  Wire.begin();         // for hardware setup

  Serial.print("Initializing SD card...");

  // -------------- CHECKS ALL HARDWARE CONNECTIONS --------------------

  // ----- Check if SD card Connected ----------------------------------
  // if (!SD.begin(BUILTIN_SDCARD)) {
  //   Serial.println("Failed to find SD Card!");
  //   while (1);
  // } // ----- END if / SD card ------------------------------------------

  // ----- Check if DSP is connected -----------------------------------
  if (!dps.begin_I2C()) {
    Serial.println("Failed to find DPS310 chip!");
    while (1);
  } // ----- END if / DPS ----------------------------------------------

  // ----- Check if HDC is connected -----------------------------------
  if (!hdc.begin(0x44, &Wire)) {
    Serial.println("Failed to find HDC302 chip");
    while (1);
  } // ----- END check HDC connection ----------------------------------

  // ------------- END CHECKS ALL HARDWARE CONNECTIONS -----------------

  Serial.println("All checks passed :)");
  delay(500); // giving time for sensor stabilization
} // END setup() -------------------------------------------------------


// ---------------------------------------------------------------------
// Code that will run repeatedly - do NOT delete
// ---------------------------------------------------------------------
void loop() {

  // ----------------------- SAVING TO SD CARD -------------------------
  // Save after a second or if RAM buffer is full
  if (millis() - lastSDTime >= SD_INTERVAL || writeIndex >= BUFF_SIZE) {
    lastSDTime = millis();  // saves current time to lastTime to be

    // ----- WRITING ---------------------------------------------------
    for (int i = 0; i < writeIndex; i++) {
        Serial.print("Index: ");
        Serial.println(i);

        // TIME
        Serial.print("Time: ");
        Serial.println(ramBuffer[i].time);

        // TEMP
        Serial.print("Teperature: ");
        Serial.print(ramBuffer[i].tempDPS);
        Serial.print(",");
        Serial.println(ramBuffer[i].tempHDC);

        // PRESSURE
        Serial.print("Pressure: ");
        Serial.println(ramBuffer[i].pressureDPS);

        // HUMIDITY
        Serial.print("Humidity: ");
        Serial.println(ramBuffer[i].humidityHDC);

        // ALTITUDE
        Serial.print("Altitude: ");
        Serial.println(ramBuffer[i].altitudeDPS);

        // ALL DATA

    } // ----- END WRITING ---------------------------------------------


    // RESETS
    writeIndex = 0;   // RESET RAM BUFFER
  } // ------------------- END SAVING TO SD CARD -----------------------

  // ------------------- READING DATA + RAM BUFFER ---------------------
  // checks if we need to write to RAM : current time - lastTime
  // writes to RAM buffer every and if RAM buffer not at max capacity
  if (millis() - lastRamTime >= RAM_INTERVAL && writeIndex < BUFF_SIZE) {
    lastRamTime = millis(); // saves current time to lastTime to be
                            // used for next comparison
    
    // ----- READING & SAVING TO RAM BUFF ------------------------------
    // TIME
    ramBuffer[writeIndex].time = millis();

    // CHIP DPS : tempature, pressure, altitude
    readDPS(writeIndex);

    // CHIP HDC : teperature, humidity
    readHDC(writeIndex);
    // ----- END READING & SAVING TO RAM BUFF --------------------------
    
    // MOVE TO NEXT INDEX 
    writeIndex++;

  } // ---------------- END READING DATA + RAM BUFFER ------------------

} // END loop() --------------------------------------------------------

// ------------------- READING DATA FUNCTIONS --------------------------

// ---------------------------------------------------------------------
// getDPS ()
// ---------------------------------------------------------------------
// Gets the following data from the DPS:
//  1. Temp
//  2. Pressure
//  3. Altitude
// and saves it to the RAM buffer
// ---------------------------------------------------------------------
void readDPS(const int index) {
  sensors_event_t event;

  dps.getTemperatureSensor()->getEvent(&event);
  ramBuffer[index].tempDPS = event.temperature;

  dps.getPressureSensor()->getEvent(&event);
  ramBuffer[index].pressureDPS = event.pressure;

  ramBuffer[index].altitudeDPS = dps.readAltitude(SEA_LEVEL);
} // END readDPS -------------------------------------------------------

// ---------------------------------------------------------------------
// readHDC ()
// ---------------------------------------------------------------------
// Gets the following data from the HDC:
//  1. Temp
//  2. Humidity
// ---------------------------------------------------------------------
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
} // END readHDC -------------------------------------------------------

// ------------------- GENERIC SD WRITE FUNCTION -----------------------
// template<typename T>
// void writeSD(File filename, T data) {
//     if (filename) {
//         filename.print(data1);
//     } 
//     else {
//         Serial.print("Error opening ");
//         Serial.println(filename);
//     }
// }
