#include <SoftwareSerial.h>

// ----------------- Pin Setup -----------------
SoftwareSerial gpsSerial(2, 3);   // GPS module: RX=2, TX=3
SoftwareSerial loraSerial(4, 5);  // LoRa module: RX=4, TX=5

String nmeaLine = "";

// ----------------- Helper Functions -----------------

// Basic NMEA checksum validation
bool nmeaChecksumOK(const String &s) {
  if (s.length() < 6) return false;
  if (s[0] != '$') return false;
  int star = s.indexOf('*');
  if (star < 0 || star + 2 >= (int)s.length()) return false;

  uint8_t sum = 0;
  for (int i = 1; i < star; i++) sum ^= (uint8_t)s[i];

  auto hexVal = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
  };
  int v1 = hexVal(s[star + 1]), v2 = hexVal(s[star + 2]);
  if (v1 < 0 || v2 < 0) return false;
  return sum == (uint8_t)((v1 << 4) | v2);
}

// Filter: only forward GGA and RMC
bool shouldForward(const String &s) {
  return s.startsWith("$GNGGA") || s.startsWith("$GNRMC");
}

// Send one payload string over RYLR998
void loraSend(const String &payload) {
  String clean = payload;
  clean.replace("\r", "");
  clean.replace("\n", "");
  loraSerial.print("AT+SEND=2,");       // destination address = 2
  loraSerial.print(clean.length());
  loraSerial.print(",");
  loraSerial.println(clean);

  // Debug output
  Serial.print("LoRa TX: ");
  Serial.println(clean);
}

// ----------------- Globals -----------------
unsigned long lastSendMs = 0;
const unsigned long SEND_INTERVAL_MS = 500; // Send at most 2 Hz

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000);  // Wait for USB Serial

  gpsSerial.begin(115200);
  loraSerial.begin(115200);

  // Configure LoRa module
  loraSerial.println("AT"); delay(200);
  loraSerial.println("AT+NETWORKID=18"); delay(200);
  loraSerial.println("AT+ADDRESS=1"); delay(200);

  Serial.println("Transmitter ready.");
}

// ----------------- Loop -----------------
void loop() {
  // Read GPS characters
  while (gpsSerial.available()) {
    char c = gpsSerial.read();

    // Start of NMEA sentence
    if (c == '$') {
      nmeaLine = "$";
      continue;
    }

    if (nmeaLine.length() == 0) continue; // ignore until we see '$'

    if (c == '\n') {
      nmeaLine.replace("\r", ""); // strip CR
      if (nmeaLine.startsWith("$") && nmeaLine.indexOf('*') > 0) {
        if (nmeaChecksumOK(nmeaLine) && shouldForward(nmeaLine)) {
          if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
            loraSend(nmeaLine);
            lastSendMs = millis();
          }
        }
      }
      nmeaLine = "";
    } else {
      nmeaLine += c;
      if (nmeaLine.length() > 120) nmeaLine = ""; // safety
    }
  }

  // Optional: read LoRa responses for debug
  while (loraSerial.available()) {
    Serial.write(loraSerial.read());
  }
}
