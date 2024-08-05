// Librerias
#include <DHT.h>
#include <HX711.h>
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>

// Variables del sensor de carga
#define DOUT A1
#define CLK A0
HX711 balanza;
float p = 0;

// Variables del sensor de humedad y temperatura
#define DHTPIN 2 
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float h,t,f = 0;
unsigned long dht_delay = 0;

// Variables del sensor de tarjetas
#define RST_PIN	9
#define SS_PIN	10
MFRC522 mfrc522(SS_PIN, RST_PIN);
byte ActualUID[4];
byte Usuario1[4]= {0x4D, 0x5C, 0x6A, 0x45} ;
byte Usuario2[4]= {0xC1, 0x2F, 0xD6, 0x0E} ;

// Variables del ServoMotor
#define SERVO_PIN 3   
#define SERVO2_PIN 4
Servo sg90;
Servo sg90_2;

// Variables del envío de datos a C Sharp
#define LED_INDICATOR 13
bool stringComplete = false;
String inputString = "";
unsigned long uart_delay = 0;

void setup() 
{
  // Incializacion de Perifericos
  Serial.begin(9600);
  pinMode(LED_INDICATOR, OUTPUT);
  digitalWrite(LED_INDICATOR, LOW);

  // Inicializacion del HX711
  balanza.begin(DOUT, CLK);
  balanza.set_scale(11605);
  balanza.tare(20); 

  // Inicializacion del RC522
  SPI.begin();
	mfrc522.PCD_Init();

  // Inicialzacion del servo SG90
  sg90.attach(SERVO_PIN);
  sg902.attach(SERVO2_PIN);

  // Inicializacion del DHT11
  dht.begin();
}

void loop() 
{
  unsigned long tiempo = millis();
  // Lectura del DHT11
  if(tiempo - dht_delay > 2000)
  {
    h = dht.readHumidity();
    t = dht.readTemperature();
    f = dht.readTemperature(true);
    dht_delay = tiempo;
  } 

  // Lectura del HX711
  p = balanza.get_units(20),3;
  p = p * -1;

  // Lectura del RC522
	if( mfrc522.PICC_IsNewCardPresent()) 
  {  
    if( mfrc522.PICC_ReadCardSerial()) 
    {
      // Enviamos serialemente su UID
      Serial.print("Card UID:");
      for (byte i = 0; i < mfrc522.uid.size; i++) {
        Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(mfrc522.uid.uidByte[i], HEX); 
        ActualUID[i] = mfrc522.uid.uidByte[i];   
      } 
      Serial.println();

      if(compareArray(ActualUID,Usuario1))
      {
        Serial.println("Acceso concedido...");
      } 
      else if(compareArray(ActualUID,Usuario2))
      {
        Serial.println("Acceso concedido...");
      } 
      else
      {
        Serial.println("Acceso denegado...");
      }

      mfrc522.PICC_HaltA();         
    }      
	}

  // Recepción de datos UART
  if(stringComplete)
  {
    inputString.trim();

    if(inputString.equals("$LF"))
    {
      digitalWrite(13, LOW);
    }
    else if(inputString.equals("$LT"))
    {
      digitalWrite(13, HIGH);
    }

    inputString = "";
    stringComplete = false;
  }

  // Activacion del Servo de la tuberia
  if(t > 55)
  {
    sg90.write(90);
  }
  else
  {
    sg90.write(0);
  }

  // Transmision de datos UART
  if(tiempo - uart_delay > 500)
  {
    Serial.println(String(h) + "," + String(t) + "," + String(p));
    uart_delay = tiempo;
  }   
}

void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char) Serial.read();
    if (inChar == '\n')
    {
      stringComplete = true;
    }
    else
    {
      inputString += inChar;
    }
  }
}

boolean compareArray(byte array1[],byte array2[])
{
  if(array1[0] != array2[0])return(false);
  if(array1[1] != array2[1])return(false);
  if(array1[2] != array2[2])return(false);
  if(array1[3] != array2[3])return(false);
  return(true);
}