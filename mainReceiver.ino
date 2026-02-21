//NEW RECIEVER 2-16-2026 10:26pm
// This is the main reciever getting from the main GPS
// THis reads nmea senteces from the Lora
// Serial 1 = Reciever
// At Network 18
// Recieve at Address 2

#include <Arduino.h>

#define LORA_SERIAL Serial1
#define BUFFER_SIZE 512

char loraLine[BUFFER_SIZE];
uint16_t idx = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000);

  LORA_SERIAL.begin(115200);
  delay(200);

  // Configure LoRa module
  LORA_SERIAL.println("AT"); delay(200);
  LORA_SERIAL.println("AT+NETWORKID=18"); delay(200);
  LORA_SERIAL.println("AT+ADDRESS=2"); delay(200);

  Serial.println("Receiver ready.");
}

void loop() {
  while (LORA_SERIAL.available()) {
    char c = LORA_SERIAL.read();
    if (c == '\n' || idx >= BUFFER_SIZE - 1) {
      loraLine[idx] = 0; // terminate

      // Debug: show raw LoRa output
      Serial.print("LoRa RAW: "); Serial.println(loraLine);

      // Parse payload if +RCV line
      if (strncmp(loraLine, "+RCV=", 5) == 0) {
        char* firstComma = strchr(loraLine, ',');
        char* secondComma = strchr(firstComma + 1, ',');
        char* lastComma = strrchr(loraLine, ',');

        if (firstComma && secondComma && lastComma && lastComma > secondComma) {
          int len = lastComma - (secondComma + 1);
          char data[BUFFER_SIZE];
          strncpy(data, secondComma + 1, len);
          data[len] = 0;

          // Replace "|" with newline for display
          for (int i = 0; i < len; i++) {
            if (data[i] == '|') data[i] = '\n';
          }

          Serial.print("Decoded NMEA:\n"); 
          Serial.println(data);
        }
      }

      idx = 0;
    } else {
      loraLine[idx++] = c;
    }
  }
}
