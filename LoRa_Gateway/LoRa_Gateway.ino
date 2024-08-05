// Frecuencia base nodo LoRa
#define FREQUENCY_868

#include <WiFi.h>
#include <LoRa_E220.h>
#include <PubSubClient.h>

// ID de nodos LoRa
#define LORA_NODE1_ID  0x00F1

//GPIO ESP32 - LoRa
#define UART2_TX  17 // ESP32 TX -- E220 RX
#define UART2_RX  16 // ESP32 RX -- E220 TX
#define AUX_PIN   13 // AUX
#define M0_PIN    4  // M0
#define M1_PIN    5  // M1
LoRa_E220 e220ttl(UART2_RX, UART2_TX, &Serial2, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_9600);

//Credenciales de red WiFi
#define WIFI_SSID   "CHESTER" 
#define WIFI_PSWD   "Manchitas"

//Credenciales de Broker MQTT
const char *mqtt_broker = "industrial.api.ubidots.com";
const char *mqtt_username = "BBUS-buVzhgr6Usllw3YbAPadWIeAWxXwew";  //Token de la cuenta de ubidots
const char *mqtt_password = NULL;
const int mqtt_port = 1883; //1883 -> sin seguridad, 8883 -> con seguridad

//ID de cliente MQTT para ESP32
const char *clientID      = "ESP32-DEV1";

//Topics de publicacion y suscripcion MQTT
const char *topic_dev_pub  = "/v1.6/devices/dev_esp32_iot";
const char *topic_sw_sub   = "/v1.6/devices/dev_esp32_iot/switch/lv";

//Prototipos de funciones
void WiFi_Scan (void);
void WiFi_Connect (void);
void MQTT_Connect(void);
void mqtt_recv_callback (char *topic, uint8_t *payload, unsigned int length);
void LoRa_Init(void);

//Objeto para menejar conexion IP para el ESP32
WiFiClient espClient;
//Objeto para menejar las conexiones y transferencias mediante MQTT
PubSubClient clientMQTT(espClient);

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


//Variables de aplicacion
char  str_json[200];
char  topic_recv[50];
char  data_recv[10];
Message_t recvMsg  = {0};
uint16_t idMessage = 0;
bool flag_data_send_ubidots = false;


//********************************************************************************************************
//********************************************************************************************************
void setup() 
{
  //Puerto Serial
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  // Configuracion de nodo LoRa
  LoRa_Init();

  //Configurar el modo de WiFi
  WiFi.mode(WIFI_STA); // Modo Estacion

  //Conectar el ESP32 al un AP
  WiFi_Connect();

  //Conectar Cliente MQTT (ESP32) al broker de ubidtos
  MQTT_Connect();
}

void loop() 
{
  //if (e220ttl.available() >= 1) 
  //{
    Serial.println("Mensaje Recibido");
		ResponseStructContainer rscRx = e220ttl.receiveMessage(sizeof(recvMsg));
    memcpy((uint8_t *)&recvMsg, (uint8_t *)rscRx.data, sizeof(recvMsg));

    sprintf(str_json, "{\"temp_c\":%.2f,\"temp_f\":%.2f,\"humedad\":%.2f,\"presion\":%.2f,\"altitud\":%.2f,\"luz\":%d}", recvMsg.tempC, recvMsg.tempF, recvMsg.humedad, recvMsg.presion, recvMsg.altitud, recvMsg.luz);
    flag_data_send_ubidots = true;

	/*	if (rscRx.status.code == 1) 
    {
			//Decodificar los datos recibidos
      Serial.println("Datos Leidos");
      idMessage = recvMsg.idNodo;

      if(idMessage == LORA_NODE1_ID)
      {
        // Formatear los datos en JSON
        sprintf(str_json, "{\"temp_c\":%.2f,\"temp_f\":%.2f,\"humedad\":%.2f,\"presion\":%.2f,\"altitud\":%.2f,\"luz\":%d}", recvMsg.tempC, recvMsg.tempF, recvMsg.humedad, recvMsg.presion, recvMsg.altitud, recvMsg.luz);
        flag_data_send_ubidots = true;
      }
		} 
    else 
    {
      Serial.println("Error al recibir datos");
		}*/
 	//}delay(2000);
  delay(4000);
  // Si se recibieron datos de un nodo, se deben publicar
  if(flag_data_send_ubidots == true)
  {
    if(WiFi.status() == WL_CONNECTED)
    {
      //Realizar conexiones a aplicaciones de internet con MQTT
      if(true == clientMQTT.connected())
      {
        flag_data_send_ubidots = false;

        //Publicacion de datos MQTT
        clientMQTT.publish(topic_dev_pub, str_json);
        Serial.println(str_json);
      }
      else
      {
        //Reconexion MQTT
        MQTT_Connect();
      }
    }
    else
    {
      //Reconcexion WiFi
      WiFi_Connect();
    }
  }

  //Mantener la conexion actualizada
  clientMQTT.loop();
}

//********************************************************************************************************
//********************************************************************************************************

//Funcion para recibir datos MQTT
void mqtt_recv_callback (char *topic, uint8_t *payload, unsigned int length)
{
  // topic  : /v1.6/devices/dev_esp32_iot/switch/lv
  // payload: 0.0. 1.0
  memcpy(topic_recv, topic, strlen(topic));
  memcpy(data_recv, (char *)payload, length);

  Serial.print("Topic recibido: ");
  Serial.println(topic_recv);
  Serial.print("Dato recibido: ");
  Serial.println(data_recv);

  if(0 == strcmp(topic_recv, topic_sw_sub))
  {
    if(0 == strcmp(data_recv, "1.0"))
    {
      //Accionar el LED del ESP32
      digitalWrite(2, HIGH);
    }
    else if(0 == strcmp(data_recv, "0.0"))
    {
      //Accionar el LED del ESP32
      digitalWrite(2, LOW);
    }
  }
}

//Funcion para conexion a broker MQTT
void MQTT_Connect(void)
{
  //Establecer la configuracion del broker MQTT
  clientMQTT.setServer(mqtt_broker, mqtt_port);

  //Establecer la funcion para recibir datos MQTT en ESP32
  clientMQTT.setCallback(mqtt_recv_callback);

  //Iniciamos y consultamos la conexion MQTT
  while(false == clientMQTT.connected())
  {
    //Evaluar si el cliente se ha conectado al broker MQTT
    if(true == clientMQTT.connect(clientID, mqtt_username, mqtt_password))
    {
      Serial.println("Cliente MQTT conectado OK");

      //El cliente debe suscribirese a los Topics para recibir informacion
      clientMQTT.subscribe(topic_sw_sub);
    }
    else
    {
      Serial.print("Cliente MQTT falla la conexion: state = ");
      Serial.println(clientMQTT.state());
      delay(2000);
    }
  }
}

void WiFi_Connect (void)
{
  //Configuracion de credenciales de red WIFI
  WiFi.begin(WIFI_SSID, WIFI_PSWD);

  //Esperar mientras se realiza la conexion del ESP32 al AP
  Serial.print("Conectando ESP32 al AP .");
  while(WL_CONNECTED != WiFi.status())
  {
    Serial.print(".");
    delay(1000);
  }

  //ESP32 esta conectado al AP
  Serial.println("");
  Serial.print("IP Local: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI    : ");
  Serial.println(WiFi.RSSI());

  Serial.println("Conexion Exitosa, bienvenido a tu APP IoT !!!");
}


void WiFi_Scan (void)
{
  int nroRedesWiFi = 0;

  //Antes de escanar redes WiFi, debo desconectar el ESP32 del AP
  WiFi.disconnect();
  delay(100);

  Serial.printf("Escaneo WiFi Iniciado... \r\n");
  //Obtener la cantidad de redes WiFi cercanas del ESP32
  nroRedesWiFi = WiFi.scanNetworks();
  Serial.printf("Escaneo Listo!! \r\n");

  if(nroRedesWiFi == 0)
  {
    Serial.printf("No existen redes disponibles :c\r\n");
  }
  else
  {
    Serial.printf("Cantidad de redes econtradas: %d \r\n", nroRedesWiFi);

    //Imprimir el SSID y RSSI de las redes encontradas
    for(int i = 0; i < nroRedesWiFi; i++)
    {
      Serial.printf("%d -> ", i+1);
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" )");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      delay(10);
    }
  }

  Serial.printf("\r\n\r\n");
  delay(5000);
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
    Serial.printf("\r\nLectura de configuracion exitosa\r\n");
  }
  else {
    Serial.printf("\r\nError en la lectura de configuracion\r\n");
  }

  // Es importante obtener el puntero de configuración antes de cualquier otra operación
  Configuration config = *(Configuration*) rc.data;

  //CONFIGURACION DE REGISTROS DEL MODULO LORA
  config.ADDH = 0x00; 
  config.ADDL = 0x1A;  

  config.CHAN = 65; // Communication channel

  config.SPED.uartBaudRate = UART_BPS_9600; // Serial baud rate
  config.SPED.airDataRate  = AIR_DATA_RATE_000_24; // Air baud rate
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

  Serial.printf("Configuracion LoRa completa \r\n\r\n");
}
