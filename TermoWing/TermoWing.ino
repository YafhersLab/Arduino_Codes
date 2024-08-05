//YawSPitchSRollSTempSHumSPresionSAltitudSBateriaSLatSLon
#include <Wire.h>
#include <DHT.h>
#include <BMP280_DEV.h>
#define pi 3.141592654

// Configuración del MPU6050
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float tiempoBucle, tiempo, tiempoAnterior;
float acelerometro[3];
float giroscopio[3];

// Configuración del DHT11
#define DHTPIN  15
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Configuración del BMP280
float temperature, preassure, altitude;
BMP280_DEV bmp280;

// Variables de los sensores
float DataYaw = 0;
float DataPitch = 0;
float DataRoll = 0;
float DataTemperatura = 0;
float DataHumedad = 0;
float DataPresion = 0;
float DataAltitud = 0;
float DataBateria = 0;
float DataLatitud = 0;
float DataLongitud = 0;

// UART
char data_UART;
int pinRX= 16; 
int pinTX = 17;

void setup() 
{
  // MPU6050 Init
  Wire.begin();                                                               
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);    

  // UART Init
	Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, pinRX, pinTX);

  // GPIO Init
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  // DHT init
  dht.begin();
  
  // BMP280 Init
  bmp280.begin(BMP280_I2C_ALT_ADDR);
  bmp280.startNormalConversion();
}

void loop() 
{
  // Obtengo el tiempo transcurrido desde el bucle anterior                                                                                                                              // almaceno el tiempo anterior
  tiempo = millis();                                                                                                                                      // almaceno el tiempo actual
  tiempoBucle = (tiempo - tiempoAnterior) / 1000;   

  // MPU6050: Calculo de los valores del Acelerometro (0x3B)
  Wire.beginTransmission(0x68);                                                                                                                     
  Wire.write(0x3B);                                                                                                                                
  Wire.endTransmission(false);                                                                                                                         
  Wire.requestFrom(0x68, 6, true);                                                                                                                     
  Acc_rawX = Wire.read() << 8 | Wire.read();                                                                                                            
  Acc_rawY = Wire.read() << 8 | Wire.read();                                                                                                            
  Acc_rawZ = Wire.read() << 8 | Wire.read();                                                                                                           
  acelerometro[0] = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * 180 / pi;                         
  acelerometro[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * 180 / pi;                     
  acelerometro[2] = atan(sqrt(pow(Acc_rawX / 16384.0,2) + pow(Acc_rawY / 16384.0, 2)) / (Acc_rawZ / 16384.0 )) * 180 / pi;

  // MPU6050: Calculo de los valores del Giroscopio (0x43)
  Wire.beginTransmission(0x68);                                                                                                                        
  Wire.write(0x43);                                                                                                                                     
  Wire.endTransmission(false);                                                                                                                         
  Wire.requestFrom(0x68, 6, true);                                                                                                                    
  Gyr_rawX = Wire.read() << 8 | Wire.read();                                                                                                   
  Gyr_rawY = Wire.read() << 8 | Wire.read();  
  Gyr_rawZ = Wire.read() << 8 | Wire.read();                                                                                                     
  giroscopio[0] = Gyr_rawX / 131.0;                                                                                                               
  giroscopio[1] = Gyr_rawY / 131.0;                                                                                                                
  giroscopio[2] = Gyr_rawZ / 131.0;

  // MPU650: Calculo de los angulos Yaw, Pitch, Roll
  DataYaw = 0.98 * (DataYaw + giroscopio[0] * tiempoBucle) + 0.02 * acelerometro[0];                                                            
  DataPitch = 0.98 * (DataPitch + giroscopio[1] * tiempoBucle) + 0.02 * acelerometro[1];                                                            
  DataRoll = 0.98 * (DataRoll + giroscopio[2] * tiempoBucle) + 0.02 * acelerometro[2];   
  
  // Lectura de sensor DHT11
  DataTemperatura = dht.readTemperature();
  DataHumedad = dht.readHumidity();

  // Lectura del BMP280
  bmp280.getMeasurements(temperature, preassure, altitude);
  DataPresion = preassure;
  DataAltitud = altitude;

  // Lectura de la Bateria
  DataBateria = random(80, 100);

  // Lectura del GPS
  DataLatitud = random(20, 60);
  DataLongitud = random(20, 60);

  // Envio de datos por UART
  if(Serial.available() > 0)
  {
    data_UART = Serial.read();
    if(data_UART == 'E')
    {
      Serial.print(DataYaw);
      Serial.print("S");
      Serial.print(DataPitch);
      Serial.print("S");
      Serial.print(DataRoll);
      Serial.print("S");
      Serial.print(DataTemperatura);
      Serial.print("S");
      Serial.print(DataHumedad);
      Serial.print("S");
      Serial.print(DataPresion);
      Serial.print("S");
      Serial.print(DataAltitud);
      Serial.print("S");
      Serial.print(DataBateria);
      Serial.print("S");
      Serial.print(DataLatitud);
      Serial.print("S");
      Serial.print(DataLongitud);
      Serial.print("S");
      delay(50);
    }
  }

  tiempoAnterior = tiempo;
}