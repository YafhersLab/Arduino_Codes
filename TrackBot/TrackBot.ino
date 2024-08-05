#include <QTRSensors.h>

//Variables para los motores TB6612FNG
const int pwmA = 3;
const int ain2 = 4;
const int ain1 = 5;
const int pwmB = 11;
const int bin1 = 6;
const int bin2 = 7;

//Variables para el QTR8A
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup(){
  // -----------CONFIGURACION TB6612FNG------------
  pinMode(pwmA, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(pwmB, OUTPUT);
  Serial.begin(9600);
  // -----------CONFIGURACION QTR8A------------
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(9);

  //---------ETAPA DE CALIBRACION-------
  calibracion();              //10 segundos
}

void loop()
{
  // desde 0 hasta 7000 guarda la posicion de la linea
  uint16_t position = qtr.readLineBlack(sensorValues);

  // imprime la lectura de cada sensor desde 0 hasta 1000
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
    }
  Serial.println(position);
  delay(250);
}

void calibracion(){
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // prendes led del Arduino para indicar que estamos calibrando

  // FORMULA PARA HALLAR EL TIEMPO DE CALIBRACION
  // 0.1 * 4 * 10 * (Numero de sensores) -> 32ms

  for (uint16_t i = 0; i < 312; i++)
  {
    qtr.calibrate();      //la funcion calibrate toma 32 ms
  }   // para calibrar 10s necesito 312 iteraciones 
  digitalWrite(LED_BUILTIN, LOW); // apagar led del arduino porque ya termine de calibrar

  // IMPRIME LOS MINIMOS VALORES OBTENIDOS EN LA LECTURA
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // IMPRIME LOS MAXIMOS VALORES OBTENIDOS EN LA LECTURA
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println(); Serial.println();
  delay(1000);
}

void motores(int vel1, int vel2){
  if(vel1 >= 0){
    analogWrite(pwmA, vel1);
    digitalWrite(ain2, 0);
    digitalWrite(ain1, 1);
  }
  
  else if(vel1 < 0){
    analogWrite(pwmA, vel1);
    digitalWrite(ain2, 1);
    digitalWrite(ain1, 0);
  }

  if(vel2 >= 0){
    analogWrite(pwmB, vel2);
    digitalWrite(bin1, 1); 
    digitalWrite(bin2, 0);
  }

  else if(vel2 < 0){
    analogWrite(pwmB, vel2);
    digitalWrite(bin1, 0); 
    digitalWrite(bin2, 1);

  }
}
