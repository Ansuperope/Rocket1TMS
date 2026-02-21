// Valores que se transmitirán
int value1, value2, value3;

// Función para mandar comandos AT al módulo LoRa
void sendCmd(String cmd)
{
  Serial1.println(cmd);  // Enviar string cmd a módulo LoRa
  delay(500);  // Esperar 500ms a que el módulo reciba el comando
  while (Serial1.available()) {
    Serial.print(char(Serial1.read()));  // Imprimir respuesta en monitor serial
  }
}

void setup() {
  Serial.begin(115200);       // Monitor serial USB
  Serial1.begin(115200);      // UART1 (RX=0, TX=1)
  delay(3000);
  Serial.println("Configurando parámetros antena LoRa");
  delay(1000);
  sendCmd("AT+ADDRESS=1");    // Configurar address
  delay(1000);
  sendCmd("AT+NETWORKID=19"); // Configurar Network ID
  delay(1000);
  sendCmd("AT+BAND?");        // Leer frecuencia
  delay(1000);
  sendCmd("AT+PARAMETER?");   // Leer parámetros
  delay(1000);
  sendCmd("AT+MODE?");        // Leer modo
  delay(1000);
}

void loop() {
  value1 = random(-100,100);
  value2 = random(0,4000);
  value3 = random(1,9);

  String data = String(value1) + "," + String(value2) + "," + String(value3);
  String datalen = String(data.length());

  Serial.println("Enviando: " + data);
  sendCmd("AT+SEND=2," + datalen + "," + data);

  delay(5000);
}
