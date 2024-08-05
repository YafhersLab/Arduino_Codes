#include <Wire.h>
#include <LoRa_E220.h>
#include <DHT.h>
#include <BMP280_DEV.h>

//Frecuencia base nodo LoRa
#define FREQUENCY_868

//Pinout DHT11
#define DHTPIN   15
#define DHTTYPE  DHT11
DHT dht(DHTPIN, DHTTYPE);

//Pinout BMP280
float temperature, preassure, altitude;
BMP280_DEV bmp280;

//Pinout Fotoresistencia
const int potPin = 34;

//GPIO ESP32 - LoRa
#define UART2_TX  17 // ESP32 TX -- E220 RX
#define UART2_RX  16 // ESP32 RX -- E220 TX
#define AUX_PIN   13 // AUX
#define M0_PIN    4  // M0
#define M1_PIN    5  // M1
LoRa_E220 e220ttl(UART2_RX, UART2_TX, &Serial2, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_9600);

//Funciones
void LoRa_Init(void);

// Estructura para el manejo de datos
typedef struct
{
  uint16_t idNodo;
  uint16_t readingId; //para monitorear la cantidad de lecturas que se manda al gateway
  float    humedad;
  float    tempC;
  float    tempF;
  float    presion;
  float    altitud;
  uint16_t luz;
}Message_t;

// Variables de aplicacion
Message_t sendMsg = {0};
uint8_t led_state = 0;

void setup() {
  // UARt0 init
	Serial.begin(115200);

  // GPIO LED
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  //DHT init
  dht.begin();
  
  //BMP280 Init
  bmp280.begin(BMP280_I2C_ALT_ADDR);
  bmp280.startNormalConversion();

  // Fotoresistencia Init
  //analogReadResolution(12);

  // Configuracion de nodo LoRa
  LoRa_Init();
  
  //Set ID Node
  sendMsg.idNodo = 0x00F1; // Puede ser cualquier ID dentro del 16 bits
}

void loop() {
  //Lectura de sensor DHT22
  sendMsg.tempC   = dht.readTemperature();      // °C
  sendMsg.tempF   = dht.readTemperature(true);  // °F
  sendMsg.humedad = dht.readHumidity();         // %

  //Lectura del BMP280
  bmp280.getMeasurements(temperature, preassure, altitude);
  sendMsg.presion = preassure + 62.53;
  sendMsg.altitud = altitude + 1006;

  //Lectura de la fotoresistencia
  uint16_t potValue = analogRead(potPin);
  sendMsg.luz = 100 - map(potValue, 1000, 2000, 0, 100);

  sendMsg.readingId++;

  //Imprimir datos
  Serial.printf("NODO 1 - Datos \r\n");
  Serial.printf("Humedad : %.2f %% \r\n", sendMsg.humedad);
  Serial.printf("Temp C  : %.2f C\r\n", sendMsg.tempC);
  Serial.printf("Temp F  : %.2f C\r\n", sendMsg.tempF);
  Serial.printf("Presion  : %.2f hPa\r\n", sendMsg.presion);
  Serial.printf("Altitud  : %.2f m\r\n", sendMsg.altitud);
  Serial.printf("Luz  : %d %%\r\n", sendMsg.luz);

  //Envio de mensaje a gateway canal 65 = 850.125 + 65 = 915.125 Mhz
  ResponseStatus rsTx = e220ttl.sendFixedMessage(0x00, 0x1A, 65, (const uint8_t *)&sendMsg, sizeof(sendMsg));

  if(rsTx.code == 1)
  {
    Serial.printf("Envio de mensaje al gateway Ok \r\n\r\n");
  }
  else
  {
    Serial.println("Envio de mensaje al gateway fallido \r\n\r\n");
  }

  // LED Toggle
  led_state = ~led_state;
  if(led_state != 0)
  {
    digitalWrite(2, HIGH);
  }
  else {
    digitalWrite(2, LOW);
  }

  delay(4000);
}


// Funcion para incializar el modulo LoRa
void LoRa_Init(void)
{
  //Variable para obtener configuracion del modulo LoRa
  ResponseStructContainer rc;

  // Inicia todos los pines y UART
	e220ttl.begin();

  //Leer configuracion actual
  rc = e220ttl.getConfiguration();

  if(rc.status.code == 1)
  {
    Serial.println("Lectura de configuracion exitosa");
  }
  else {
    Serial.println("Error en la lectura de configuracion");
  }

  // Es importante obtener el puntero de configuración antes de cualquier otra operación
  Configuration config = *(Configuration*) rc.data;

  //CONFIGURACION DE REGISTROS DEL MODULO LORA
  config.ADDH = 0x00; 
  config.ADDL = 0x4A;  

  config.CHAN = 65; // Communication channel = 850.125 + 65 = 915.125 Mhz

  config.SPED.uartBaudRate = UART_BPS_9600; // Serial baud rate
  config.SPED.airDataRate  = AIR_DATA_RATE_000_24; // Air baud rate 2.4Kbps
  config.SPED.uartParity   = MODE_00_8N1; // Parity bit

  config.OPTION.subPacketSetting = SPS_200_00; // Packet size
  config.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED; // Need to send special command
  config.OPTION.transmissionPower = POWER_22; // Device power

  config.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED; // Enable RSSI info
  config.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION; //FT_TRANSPARENT_TRANSMISSION

  config.TRANSMISSION_MODE.enableLBT = LBT_DISABLED; // Check interference
  config.TRANSMISSION_MODE.WORPeriod = WOR_2000_011; // WOR timing

  //Establecer la configuracion del modulo LoRa
  e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
  rc.close(); //Terminar la configuracion

  Serial.println("Configuracion completa");
}
