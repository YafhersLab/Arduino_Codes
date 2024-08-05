//Includes
#include "Simple_MPU6050.h"

//Defines
#define MPU6050_DEFAULT_ADDRESS 0x68
#define OFFSETS  -1678,     170,    3764,      50,       5,     -13
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);

// ########################################  INCLUDES   #############################################//
#include <Arduino.h>
#include <Wire.h>
Simple_MPU6050 mpu;

// ########################################  DEFINES   #############################################//
#define pi 3.141592654

// ########################################  VARIABLES   #############################################//
//Variables de los motores nema
int step1 = 4;
int dir1 = 3;
int step2 = 6;
int dir2 = 5;

// Variables de temporizacion
float tiempoBucle, time, tiempoAnterior;
float yaw = 0, pitch = 0, roll = 0;

// Variables del PID
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
double kp = 2;                                                                                                                                         
double ki = 0;                                                                                                                                          
double kd = 4;                                                                                                                                          
double velocidadBase = 1300;
float anguloDeseado = 0;
int frecuencia;

void MPU6050(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {	
  uint8_t SpamDelay = 100;			// demora para escribir en monitor serie de 100 mseg
  Quaternion q;					// variable necesaria para calculos posteriores
  VectorFloat gravity;				// variable necesaria para calculos posteriores
  float ypr[3] = { 0, 0, 0 };			// array para almacenar valores de yaw, pitch, roll
  float xyz[3] = { 0, 0, 0 };			// array para almacenar valores convertidos a grados de yaw, pitch, roll
  spamtimer(SpamDelay) {			// si han transcurrido al menos 100 mseg entonces proceder
    mpu.GetQuaternion(&q, quat);		// funcion para obtener valor para calculo posterior
    mpu.GetGravity(&gravity, &q);		// funcion para obtener valor para calculo posterior
    mpu.GetYawPitchRoll(ypr, &q, &gravity);	// funcion obtiene valores de yaw, ptich, roll
    mpu.ConvertToDegrees(ypr, xyz);		// funcion convierte a grados sexagesimales
    yaw = xyz[0] + 15;
    pitch = xyz[1];
    roll = xyz[2];
  }
}

// ########################################  SETUP   #############################################//
void setup() {
  // Primera comunicacion con el MPU6050
  Wire.begin();                                                                                                                                         // iniciar comunicacion serial I2C
  Wire.setClock(400000);
  //MPU6050
  mpu.SetAddress(0x68).load_DMP_Image(OFFSETS);
  mpu.on_FIFO(MPU6050);

  //Configuración del Timer2
  pinMode(13, OUTPUT);
  TCCR1A = 0;                                                                                                                                           // limpiamos el registro de control A del timer 1
  TCCR1B = 0;                                                                                                                                           // limpiamos el registro de control B del timer 1
  TCCR1B |= B00000100;                                                                                                                                  // prescaler a 256
  TIMSK1 |= B00000010;                                                                                                                                  // habilito la interrupcion por comparacion
  OCR1A = 31250;                                                                                                                                        // valor calculado para una interrupcion cada 500ms

  // Iniciamos perifericos a usar en el programa
  Serial.begin(9600);     Serial.println("Go");                                                                                                         // inicio el monitor serial  a 9600 baudios                                                                                                                              // adjunto el pin 5 al motor izquierdo
  time = millis();              

  // Configuramos los pines GPIO
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(step1,OUTPUT);
  pinMode(step2,OUTPUT);                                                                                                                                // inicio el conteo del millis
}

// ########################################  LOOP   #############################################//
void loop(){
  // Obtengo el tiempo transcurrido desde el bucle anterior
  tiempoAnterior = time;                                                                                                                                // almaceno el tiempo anterior
  time = millis();                                                                                                                                      // almaceno el tiempo actual
  tiempoBucle = (time - tiempoAnterior) / 1000;                                                                                                         // obtengo el tiempo transcurrido en segundos

  mpu.dmp_read_fifo();		
  String imprimeme = "Inclinación: " + String(yaw) + "\t PID valor: " + PID + " \t frecuencia: " + frecuencia + "\t " + (time - tiempoAnterior);
  Serial.println(imprimeme);                                                                                                                           // imprimo el angulo leido
  
  // Ahora comencemos con el diseño del controlador PID con en angulo Y como referencia
  error = abs(yaw - anguloDeseado);                                                                                                             // calculo el error
  pid_p = kp * error;                                                                                                                                  // creamos el valor proporcional

  // La parte integral se activa para corregir el valor en estado estable por ello se activa de -3 a 3
  if (-5 < error || error > 5){
    pid_i = pid_i + (ki * error * tiempoBucle);                                                                                                        // creamos el valor integral
  }

  // La parte derivada seria el error menos el error anterior
  pid_d = kd * ((error - previous_error) / tiempoBucle);                                                                                               // creamos el valor derivativo

  // Valor final del PID
  PID = pid_p + pid_i + pid_d;

  // Finalmente escribo los valores en los motores
  frecuencia = constrain(300 - PID, 70, 300);
  OCR1A = frecuencia;

  previous_error = error;
}

ISR(TIMER1_COMPA_vect){
  TCNT1 = 0;         

  // Correcion de direccion
  if (yaw  > 0){
    digitalWrite(dir1, 1);   digitalWrite(dir2, 0);
  }
  if(yaw < 0){
    digitalWrite(dir1, 0);   digitalWrite(dir2, 1);
  }

  // Generacion de pasos                    
  digitalWrite(step1, !(digitalRead(step1)));   digitalWrite(step2, !(digitalRead(step2)));
}
