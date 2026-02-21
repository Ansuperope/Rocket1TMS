// Gianni and Kyle Code
// This is for the main gps -> main reciever
// Gets RMC and GGA nmea lines
// Serial 7 = GPS
// Serial 2 = lora
// At Network 18
// Send at Address 2

// START OF CODE
String nmeaLine = "";

// helper functions to check full payload line to prevent buffering issue (payload data sending too fast)
// Basic NMEA checksum validation (optional but very helpful)
bool nmeaChecksumOK(const String &s) {
  if (s.length() < 6) return false;
  if (s[0] != '$') return false;
  int star = s.indexOf('*');
  if (star < 0 || star + 2 >= (int)s.length()) return false;
  uint8_t sum = 0;
  for (int i = 1; i < star; i++) sum ^= (uint8_t)s[i];
  // Parse two hex chars after '*'
  char h1 = s[star + 1];
  char h2 = s[star + 2];
  auto hexVal = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
  };
  int v1 = hexVal(h1), v2 = hexVal(h2);
  if (v1 < 0 || v2 < 0) return false;
  uint8_t expected = (uint8_t)((v1 << 4) | v2);
  return sum == expected;
}

// Filter: send only the sentences you actually want over LoRa
// Options include: GNGGA, GPGGA, GNRMC, GPRMC, GNGLL, GPGLL, GNZDA, GPZDA
bool shouldForward(const String &s) {
  if (s.startsWith("$GNGGA")) return true;
  if (s.startsWith("$GNRMC")) return true;
  return false;
}

// Send one payload string over RYLR998 AT
void loraSend(uint16_t destination, const String &payload) {
  // RYLR998 payload must not include CR/LF; strip them
  String clean = payload;
  clean.replace("\r", "");
  clean.replace("\n", "");

  //The RYLR998 AT format is: AT+SEND=<destination address>,<length>,<data>
  Serial2.print("AT+SEND=");
  Serial2.print(destination);
  Serial2.print(",");
  Serial2.print(clean.length());
  Serial2.print(",");
  Serial2.println(clean);

  // Optional debug to USB
  Serial.print("LoRa TX: ");
  Serial.println(clean);
}

//How to stop ERR=5 (practical solution)
  // Add rate limiting so you don’t send faster than LoRa can handle.
  // Example: send at most once every 500 ms (2 Hz) or even 1 Hz to start.
unsigned long lastAnySendMs = 0;
const unsigned long SEND_INTERVAL_MS = 1000; // 1 packet/sec
bool toggle = false; // alternate between GGA and RMC
String lastGGA = "";
String lastRMC = "";

void setup() {
  Serial.begin(115200);

  // “Wait until the USB Serial connection is ready, but don’t wait longer than 4 seconds.”
  while (!Serial && millis() < 4000);

  // GNSS
  Serial7.begin(115200);
  delay(200);

  // LoRa (set baud to whatever your RYLR998 is actually using)
  Serial2.begin(115200);
  delay(200);

  // Configure LoRa module (TX node)
  // This is configuration, not sending data.
  // Each line is a command sent to the LoRa module over UART:
    // AT
    // “Are you alive?” → module should reply OK.
    // AT+NETWORKID=18
    // Sets which “network” the module belongs to.
    // Only modules with the same NETWORKID will talk to each other.
    // AT+ADDRESS=1
    // Sets this module’s node address to 1.
    // (So other nodes can send to it, and it can be identified as the sender.)
  Serial2.println("AT");
  delay(200);
  Serial2.println("AT+NETWORKID=18");
  delay(200);
  Serial2.println("AT+ADDRESS=1");
  delay(200);

  Serial.println("Transmitter ready.");
}

void loop() {
  while (Serial7.available()) {
    char c = (char)Serial7.read();
    // Resync: if we see '$', that's the start of a new NMEA sentence.
    if (c == '$') {
      nmeaLine = "$";
      continue;
    }
    // If we haven't seen a '$' yet, ignore bytes (we're not aligned)
    if (nmeaLine.length() == 0) {
      continue;
    }
    if (c == '\n') {
      // End of sentence (strip CR just in case)
      nmeaLine.replace("\r", "");
      // Only send if it looks like a complete NMEA sentence
      if (nmeaLine.startsWith("$") && nmeaLine.indexOf('*') > 0) {
        if (nmeaChecksumOK(nmeaLine) && shouldForward(nmeaLine)) {
          if (nmeaLine.startsWith("$GNGGA")) lastGGA = nmeaLine;
            else if (nmeaLine.startsWith("$GNRMC")) lastRMC = nmeaLine;
            // your loraSend should strip \r\n too
        }
        

      }
      nmeaLine = "";
    } else {
      // Append normal characters
      nmeaLine += c;
      // Safety: avoid runaway memory if something goes wrong
      if (nmeaLine.length() > 120) {
        nmeaLine = "";
      }
    }
  }
  if (millis() - lastAnySendMs >= SEND_INTERVAL_MS) {
          if (!toggle && lastGGA.length() > 0) {
            loraSend(2, lastGGA);
            toggle = true;
            lastAnySendMs = millis();
          } else if (toggle && lastRMC.length() > 0) {
            loraSend(2, lastRMC);
            toggle = false;
            lastAnySendMs = millis();
          }
        }

  // Read LoRa responses (debug)
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
}
