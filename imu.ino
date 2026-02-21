// BNO -> GUN
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>

// ===== Objects =====
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_GPS GPS(&Serial8);

// ===== Conversion constants =====
#define KNOTS_TO_FPS 1.68781
#define METERS_TO_FEET 3.28084

// ===== LoRa AT command function =====
void sendCmd(const char* cmd) {
  Serial1.println(cmd);
  delay(300);
  while (Serial1.available()) {
    Serial.print((char)Serial1.read());
  }
}

// ===== Low-pass filter variables =====
float alpha = 0.1;
float accX_smooth = 0, accY_smooth = 0, accZ_smooth = 0;

// ===== Store last valid GPS values =====
int lastHour = 0, lastMinute = 0, lastSecond = 0;
float lastVelocity = 0, lastAltitude = 0;
float lastLatitude = 0, lastLongitude = 0;

// ===== LoRa timing =====
unsigned long lastSend = 0;
const unsigned long sendInterval = 1000; // 1 Hz

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial8.begin(9600);

  delay(3000);
  Serial.println("Starting system...");

  // ===== GPS setup =====
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);

  // ===== IMU setup =====
  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  delay(1000);

  // ===== LoRa setup =====
  sendCmd("AT+ADDRESS=1");
  sendCmd("AT+NETWORKID=19");

  Serial.println("System ready.");
}

void loop() {

  // ==========================
  // CONTINUOUS GPS READING
  // ==========================
  char c = GPS.read();
  if (c) Serial.write(c);  // raw NMEA debug

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;  // wait for next sentence

    if (GPS.fix) {

      lastHour = GPS.hour - 8;  // Pacific time
      if (lastHour < 0) lastHour += 24;

      lastMinute = GPS.minute;
      lastSecond = GPS.seconds;

      lastVelocity = GPS.speed * KNOTS_TO_FPS;
      lastAltitude = GPS.altitude * METERS_TO_FEET;
      lastLatitude = GPS.latitudeDegrees;
      lastLongitude = GPS.longitudeDegrees;

      Serial.println("\nGPS FIX ACQUIRED");
    } else {
      Serial.println("\nNO GPS FIX");
    }

    Serial.print("Satellites: ");
    Serial.println(GPS.satellites);
  }

  // ==========================
  // IMU READING
  // ==========================
  sensors_event_t raw;
  bno.getEvent(&raw, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  float accX = raw.acceleration.x - gravity.x();
  float accY = raw.acceleration.y - gravity.y();
  float accZ = raw.acceleration.z - gravity.z();

  accX_smooth = alpha * accX + (1 - alpha) * accX_smooth;
  accY_smooth = alpha * accY + (1 - alpha) * accY_smooth;
  accZ_smooth = alpha * accZ + (1 - alpha) * accZ_smooth;

  float threshold = 0.05;
  if (fabs(accX_smooth) < threshold) accX_smooth = 0;
  if (fabs(accY_smooth) < threshold) accY_smooth = 0;
  if (fabs(accZ_smooth) < threshold) accZ_smooth = 0;

  // ==========================
  // 1 Hz LoRa Transmission
  // ==========================
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    char data[150];
    int datalen = snprintf(data, sizeof(data),
                           "%02d:%02d:%02d,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f",
                           lastHour, lastMinute, lastSecond,
                           lastVelocity,
                           lastLatitude,
                           lastLongitude,
                           lastAltitude,
                           accX_smooth, accY_smooth, accZ_smooth);

    Serial.println("\n--- Sending Packet ---");
    Serial.println(data);
    Serial.println("----------------------");

    Serial1.print("AT+SEND=2,");
    Serial1.print(datalen);
    Serial1.print(",");
    Serial1.println(data);
  }
}
