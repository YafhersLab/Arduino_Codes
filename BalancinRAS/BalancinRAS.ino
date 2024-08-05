//###Librerias
#include "Simple_MPU6050.h"
#define OFFSETS  -5114,     484,    1030,      46,     -14,       6
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);

Simple_MPU6050 mpu;



void mostrar_valores (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {	
  //Demora para escribir en monitor serie
  uint8_t SpamDelay = 100;		
  //Variables para calculos
  Quaternion q;					
  VectorFloat gravity;			
  //Array para almacenar angulos
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };	
  spamtimer(SpamDelay) {
    mpu.GetQuaternion(&q, quat);  mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);	
    Serial.printfloatx(F("Yaw")  , xyz[0], 9, 4, F(",   "));  // muestra en monitor serie rotacion de eje Z, yaw
    Serial.printfloatx(F("Pitch"), xyz[1], 9, 4, F(",   "));  // muestra en monitor serie rotacion de eje Y, pitch
    Serial.printfloatx(F("Roll") , xyz[2], 9, 4, F(",   "));  // muestra en monitor serie rotacion de eje X, roll
    Serial.println();				// salto de linea
  }
}

void setup() {
  speedI2C();
  Serial.begin(115200);
  Serial.println(F("Go"));
  mpu.SetAddress(0x68).load_DMP_Image(OFFSETS);
  mpu.on_FIFO(mostrar_valores);
}

void loop() {
  mpu.dmp_read_fifo();
}

void speedI2C(){
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE	// activacion de bus I2C a 400 Khz
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}