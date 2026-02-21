// Se definen los pines para utilizar el Serial2
#define RXD2 16
#define TXD2 17

int varInMsg = 7;  // Variables que estarán en el mensaje recibido
int commasInMsg = varInMsg - 1;  // Calcular las comas que habrá en el mensaje
int keyPos[10];  // Array para almacenar la posición de las comas en el mensaje

// Strings donde guardaremos las variables del mensaje recibido
String msg;
String txId;
String dataLen;
String value1;
String value2;
String value3;
String rssi;
String snr;

// Función para mandar comandos AT al módulo LoRa
void sendCmd(String cmd)
{
  Serial2.println(cmd);  // Enviar string cmd a módulo LoRa
  delay(100);  // Esperar 500ms a que el módulo reciba el comando
  while (Serial2.available()) {
    Serial.print(char(Serial2.read()));  // Si hay respuesta imprimirla en el monitor serial 
  }
}

void setup() {
  Serial.begin(115200);  // Iniciar el monitor serial a 115200 baudios
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // Iniciar el Serial2 a 115200
  delay(3000);
  Serial.println("Configurando parámetros antena LoRa");
  delay(1000);
  sendCmd("AT+ADDRESS=2");  // Configurar el address a 2
  delay(1000);
  sendCmd("AT+NETWORKID=19");  // Configurar el Network ID a 19
  delay(1000);
  sendCmd("AT+BAND?");  // Leer la frecuencia configurada
  delay(1000);
  sendCmd("AT+PARAMETER?");  // Leer los parámetros configurados
  delay(1000);
  sendCmd("AT+MODE?");  // Leer el modo configurado
  delay(1000);
}

void loop() { //Choose Serial1 or Serial2 as required
  // Esperar por mensaje del transmisor
  while (Serial2.available()) {
    msg = Serial2.readString();
  }
  msg.trim();  // Eliminar espacios antes y después del mensaje

  // Función para detectar las comas del mensaje recibido
  for(int i = 1; i <= commasInMsg; i++)
  {
    keyPos[i] = msg.indexOf(',', keyPos[i-1] + 1);
  }

  // Obtener el valor de las variables del mensaje
  txId = msg.substring(5, keyPos[1]);
  dataLen = msg.substring(keyPos[1] + 1, keyPos[2]);
  value1 = msg.substring(keyPos[2] + 1, keyPos[3]);
  value2 = msg.substring(keyPos[3] + 1, keyPos[4]);
  value3 = msg.substring(keyPos[4] + 1, keyPos[5]);
  rssi = msg.substring(keyPos[5] + 1, keyPos[6]);
  snr = msg.substring(keyPos[6] + 1);

  // Imprimir el mensaje recibido del transmisor
  Serial.println(msg);

  // Imprimir la posición de las comas del mensaje
  Serial.print("Posición de las comas:");
  for(int i = 1; i <= commasInMsg; i++)
  {
    Serial.print(keyPos[i]);
    Serial.print(" ");
  }

  // Imprimir las variables del mensaje
  Serial.println(" ");
  Serial.print("TxId: " + txId);
  Serial.print(" Length: " + dataLen);
  Serial.print(" Value 1: " + value1);
  Serial.print(" Value 2: " + value2);
  Serial.print(" Value 3: " + value3);
  Serial.print(" RSSI: " + rssi);
  Serial.println(" SNR: " + snr);
  delay(1000);
}
