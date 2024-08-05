#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <stdio.h> //sprintf()
#include <string.h> //manejo de strings
#include <stdint.h>

//Pinout sensor DHT22
#define DHT_PIN 4
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

//Credenciales de red WiFi
#define WIFI_SSID   "CHESTER" 
#define WIFI_PSWD   "Manchitas"

//Credenciales de Broker MQTT
const char *mqtt_broker = "industrial.api.ubidots.com";
const char *mqtt_username = "BBUS-buVzhgr6Usllw3YbAPadWIeAWxXwew";  //Token de la cuenta de ubidots
const char *mqtt_password = NULL;
const int mqtt_port = 1883; //1883 -> sin seguridad, 8883 -> con seguridad

//ID de cliente MQTT para ESP32
const char *clientID = "ESP32-DEV1";

//Topics de publicacion y descripcion para MQTT
const char *topic_dev_pub = "/v1.6/devices/dev_esp32_iot";
const char *topic_sw_sub = "/v1.6/devices/dev_esp32_iot/switch/lv";

//Prototipos de funciones
void WiFi_Scan (void);
void WiFi_Connect (void);
void MQTT_Connect(void);
void mqtt_recv_callback(char *topic, uint8_t *payload, unsigned int length);

//Objeto para manejar conexion IP para el ESP32
WiFiClient espClient;
//Objeto para manejar las conexiones y transferencias mediante MQTT
PubSubClient clientMQTT(espClient);

//Variables de aplicacion
float tempF;
float tempC;
float hmd;
char str_json[100];
char topic_recv[50];
char data_recv[10];

void setup() 
{
  //Puerto Serial
  Serial.begin(115200);

  //DHT22 Init
  dht.begin();

  //Configurar el modo de WiFi
  WiFi.mode(WIFI_STA); // Modo Estacion

  //Conectar el ESP32 al un AP
  WiFi_Connect();

  //Conectar cliente MQTT (esp32) al broker de Ubidots
  MQTT_Connect();
}

void loop() 
{
  //Evaluar conexion WiFi
  if(WL_CONNECTED == WiFi.status())
  {
    //Evaluar conexion MQTT
    if(true == clientMQTT.connected())
    {
      //Podemos publicar los datos en Ubidots
      //Leer datos del sensor
      hmd = dht.readHumidity();
      tempC = dht.readTemperature();
      tempF = dht.readTemperature(true);

      //Armar los datos en formato JSON
      sprintf(str_json, "{\"temp_c\":%.2f,\"temp_f\":%.2f,\"humedad\":%.2f}", tempC, tempF, hmd);

      //Publicar data
      clientMQTT.publish(topic_dev_pub, str_json);

      //Lo muestro en el monitor serial
      Serial.printf("Data: %s \r\n", str_json);
    }
    else
    {
      //Realizar una reconexion MQTT
      MQTT_Connect();
    }

    //Mantener la conexion actualizada
    clientMQTT.loop();

  }
  else
  {
    //Realizar una reconexion WiFi
    WiFi_Connect();
  }

  delay(5000);
}

//Desarrollo de funciones
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

  Serial.print("Conexion Exitosa, bienvenido a tu APP IoT !!!");
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
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      delay(10);
    }
  }

  Serial.printf("\r\n\r\n");
  delay(5000);
}

//Funcion para conexion a broker MQTT
void MQTT_Connect(void)
{
  //Establecer la configuracion del broker
  clientMQTT.setServer(mqtt_broker, mqtt_port);

  //Establecer la funcion que va a recibir datos MQTT en ESP32
  clientMQTT.setCallback(mqtt_recv_callback);

  //Consultamos la conexion MQTT
  while(false == clientMQTT.connected())
  {
    //Evaluar si el cliente se ha conectado al broker MQTT
    if(true == clientMQTT.connect(clientID, mqtt_username, mqtt_password))
    {
      Serial.println("Cliente MQTT conectado OK");

      //El cliente debe suscribirse a los topics para recibir informacion
      clientMQTT.subscribe(topic_sw_sub);
    }
    else{
      Serial.print("Cliente MQTT falla la conexion: state = ");
      Serial.println(clientMQTT.state());
      delay(2000);
    }
  }
}

//Funcion para recibir los datos del broker de Ubidots
void mqtt_recv_callback(char *topic, uint8_t *payload, unsigned int length){
  memcpy(topic_recv, topic, strlen(topic));
  memcpy(data_recv, payload, length);

  Serial.print("Dato recibido: ");
  Serial.println(data_recv);

  if(0 == strcmp(topic_recv, topic_sw_sub))
  {
    if(0 == strcmp(data_recv, "1,0"))
    {
      //Accionar el led del ESP32
      digitalWrite(2, HIGH);
    }
    else if(0 == strcmp(data_recv, "0.0"))
    {
      //Apagar el led del ESP32
      digitalWrite(2, LOW);
    }
  }
}