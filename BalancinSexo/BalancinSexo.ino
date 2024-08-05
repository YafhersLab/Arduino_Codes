//Librerias a utilizar
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

//variables del motor
int step1 = 4;
int dir1 = 3;
int step2 = 6;
int dir2 = 5;
float k = 0;

//variables del mpu


MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//variables que sirven
bool mpuInterrupt = false;
Quaternion q;          
VectorFloat gravity;    
float ypr[3];    
float pitch;
float a_pt;

void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
  //pines de los motores
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(step1,OUTPUT);
  pinMode(step2,OUTPUT);

 //COMUNICACION I2C CON EL MPU
 //iniciar dispositivos
  Serial.begin(9600);
  Serial.println("Iniciando dispositivos I2C...");
  Wire.begin();
  mpu.initialize();

  //verificar conexion
  Serial.println(F("Testeando conexion a los dispositivos..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 conectado :D") : F("MPU6050 no conectado D:"));

  //activar pulsador para iniciar programa
  Serial.println(F("\nEscribe algo: "));
  while (Serial.available() && Serial.read());
  while (!Serial.available());                
  while (Serial.available() && Serial.read());

  //carga configuracion en DMP
  Serial.println(F("Iniciando DMP..."));
  devStatus = mpu.dmpInitialize();

  //offsets
  mpu.setXGyroOffset(136);
  mpu.setYGyroOffset(5);
  mpu.setZGyroOffset(3);
  mpu.setZAccelOffset(2896); 

  //estado del dmp
  if (devStatus == 0) {
      Serial.println(F("Activando DMP..."));
      mpu.setDMPEnabled(true);
      Serial.println(F("Activando interrupcion del arduino"));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      Serial.println(F("DMP listo, esperado interrupcion"));
      dmpReady = true;
      //obtener el tamaño esperado del paquete DMP para su posterior comparación
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      Serial.print(F("DMP no conectado correctamente"));
  }



}

void loop() {
  //prendemos motores nema
  a_pt = pitch;
  lecturaMPU();
  //motores(180,650);
  //delay(100);
  // motores(-180,650);
  // delay(10);
  if (abs(a_pt-pitch)<20 && abs(pitch)<90)
    PID(1000,0.00000,0);
}

void motores(float ang, int vel){
  int pasos = ang * 200. / 360.;
  int delay = vel;
  //indicamos direccion
  if(ang >= 0){
    digitalWrite(dir1,HIGH);  digitalWrite(dir2,LOW);
  }
  else{
    digitalWrite(dir1,LOW);   digitalWrite(dir2,HIGH);
    pasos *= -1;
  } 
  //mandamos la señal cuadrada
  for(int i=0; i < pasos; i++){
    digitalWrite(step1, 1);   digitalWrite(step2, 1);
    delayMicroseconds(delay);
    digitalWrite(step1, 0);   digitalWrite(step2, 0);
    delayMicroseconds(delay);
  }
}


void lecturaMPU(){
// if programming failed, don't try to do anything
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available
  

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      pitch = ypr[1] * 180/M_PI - 60;
      Serial.print(pitch);

      if(abs(pitch)<5.){
      Serial.println("  Equilibrado");
      }
      else{Serial.println("  Desequilibrado");}

  }
}

float err_ant=0;
void PID(float kp, float kd, float ki){
  float err = pitch;  

  int pasos = err * 200./360.;
  //int delay = constrain(kp*err + kd*(err-err_ant),650,3000);
  //delay = map(delay,400,3000,2500,400);/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int delay = 650;
  //indicamos direccion
  if(err <= 0){
    digitalWrite(dir1,HIGH);  digitalWrite(dir2,LOW);
    pasos = pasos*(-1) +1;
  }
  else{
    digitalWrite(dir1,LOW);   digitalWrite(dir2,HIGH);
    pasos +=1;
  } 
  //mandamos la señal cuadrada
  for(int i=0; i < pasos; i++){
    digitalWrite(step1, 1);   digitalWrite(step2, 1);
    delayMicroseconds(delay);
    digitalWrite(step1, 0);   digitalWrite(step2, 0);
    delayMicroseconds(delay);
  }
  err_ant = err;

} 


 