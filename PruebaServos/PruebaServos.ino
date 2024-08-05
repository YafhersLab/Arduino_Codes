#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

//Confii
const char* ssid = "CHESTER";
const char* password = "Manchitas";

//Configuracion del MQTT
WiFiClient espClient;
PubSubClient client(espClient);

//Configuracion del DHT11
#define DHTTYPE DHT11
#define DHTPIN 14
DHT dht(DHTPIN, DHTTYPE);

//Variables para las mediciones
long lastMeasure = 0;

//Configuración del Driver PCA9685
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x41);

unsigned int pos0[6] = {120,85,105,105,120,140}; 
unsigned int pos180[6] = {463,420,455,440,460,480}; 

void setup() {
  //Iniciamos el PCA9685
  servos.begin();
  servos.setOscillatorFrequency(27000000);
  servos.setPWMFreq(50);

  //Iniciamos la comunicacion MQTT
  setup_wifi();
  client.setServer("192.168.18.6", 1883);
  client.setCallback(callback);

  //Iniciamos el monitor Serial
  Serial.begin(9600);
  pinMode(2, OUTPUT);
}

void loop() {

  setServo(0,90);
  delay(1000);
  setServo(0,180);
  delay(1000);
  /*if (!client.connected()) {
    reconnect();
  }
  if (!client.loop())
    client.connect("ESP32Client");

  long now = millis();
  if (now - lastMeasure > 5000) {
    lastMeasure = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float f = dht.readTemperature(true);

    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println("Fallo al leer el sensor DHT!");
      return;
    }

    float hic = dht.computeHeatIndex(t, h, false);
    static char temperatureTemp[7];
    dtostrf(hic, 6, 2, temperatureTemp);

    static char humidityTemp[7];
    dtostrf(h, 6, 2, humidityTemp);

    client.publish("yafhers/temperatura", temperatureTemp);
    client.publish("yafhers/humedad", humidityTemp);
  }*/
}

//Funcion que configura la posicion del servo
void setServo(uint8_t n_servo, int angulo) {
  int duty = map(angulo,0,180,pos0[n_servo], pos180[n_servo]);
  servos.setPWM(n_servo, 0, duty); 
}

// Funcion que interpreta la informacion que llega al broker
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("] ");
  String valor;

  for (int i = 0; i < length; i++) {
    valor += (char)payload[i];
  }
  Serial.println();

  if (strcmp(topic, "yafhers/servo0") == 0) {
    setServo(0, valor.toInt());
  }
  if (strcmp(topic, "yafhers/servo1") == 0) {
    setServo(1, valor.toInt());
  }
  if (strcmp(topic, "yafhers/servo2") == 0) {
    setServo(2, valor.toInt());
  }
  if (strcmp(topic, "yafhers/servo3") == 0) {
    setServo(3, valor.toInt());
  }
  if (strcmp(topic, "yafhers/servo4") == 0) {
    setServo(4, valor.toInt());
  }
  if (strcmp(topic, "yafhers/servo5") == 0) {
    setServo(5, valor.toInt());
  }
}

//Funcion que se reconecta al Broker
void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conectar a MQTT...");

    if (client.connect("ESP32Client")) {
      Serial.println("conectado");
      digitalWrite(2, HIGH);

      client.subscribe("yafhers/servo0");
      client.subscribe("yafhers/servo1");
      client.subscribe("yafhers/servo2");
      client.subscribe("yafhers/servo3");
      client.subscribe("yafhers/servo4");
      client.subscribe("yafhers/servo5");

    } else {
      Serial.print("fallo, rc=");
      Serial.print(client.state());
      Serial.println(" volveremos a conectar en 5 segundos");
      digitalWrite(2, LOW);
      delay(5000);
    }
  }
}

//Funcion que configura el WiFi
void setup_wifi() {
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi conectado :D");
  Serial.println("Direccion IP: ");
  Serial.println(WiFi.localIP());
}