// ########################################  INCLUDES   #############################################//
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// ########################################  DEFINES   #############################################//
#define pi 3.141592654

// ########################################  VARIABLES   #############################################//
//  Definicion de los motores
Servo right_prop;
Servo left_prop;

// En el datasheet del MPU6050, la IMU nos entrega 3 valores de 16 bits para la acelaración y 3 para el giro
// estos valores nos lo entrega en 8 bits por lo que tendremos que unirnos para guardar todo en una sola variable
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float acelerometro[2];
float giroscopio[2];
float angulos[2];

// Variables de temporizacion
float tiempoBucle, time, tiempoAnterior;
int i;

// Variables del PID
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
double kp = 3.55;                                                                                                                                         
double ki = 0.005;                                                                                                                                          
double kd = 2.05;                                                                                                                                          
double velocidadBase = 1300;
float anguloDeseado = 0;

// ########################################  SETUP   #############################################//
void setup() {
  // Primera comunicacion con el MPU6050
  Wire.begin();                                                                                                                                         // iniciar comunicacion serial I2C
  Wire.beginTransmission(0x68);                                                                                                                         // nos conectamos al 0x68 (MPU6050)
  Wire.write(0x6B);                                                                                                                                     // nos vamos al registro 0x6B
  Wire.write(0);                                                                                                                                        // seteo el bit 1, indico que se va a usar el reloj interno del Arduino a 8Mhz
  Wire.endTransmission(true);                                                                                                                           // finaliza la comunicacion I2C

  // Iniciamos perifericos a usar en el programa
  Serial.begin(9600);                                                                                                                                   // inicio el monitor serial  a 9600 baudios
  right_prop.attach(3);                                                                                                                                 // adjunto el pin 3 al motor derecho
  left_prop.attach(5);                                                                                                                                  // adjunto el pin 5 al motor izquierdo
  time = millis();                                                                                                                                      // inicio el conteo del millis

  // Para iniciar los driver ESC debes enviar un valor minimo de PWM, sino no van a encender
  left_prop.writeMicroseconds(1000);
  right_prop.writeMicroseconds(1000);
  delay(7000);                                                                                                                                          // espera de 7 segundos para esperar que los ESC inicien
}

// ########################################  LOOP   #############################################//
void loop(){
  // Obtengo el tiempo transcurrido desde el bucle anterior
  tiempoAnterior = time;                                                                                                                                // almaceno el tiempo anterior
  time = millis();                                                                                                                                      // almaceno el tiempo actual
  tiempoBucle = (time - tiempoAnterior) / 1000;                                                                                                         // obtengo el tiempo transcurrido en segundos

  // Obtenemos los valores del acelerometro (0x3B)
  Wire.beginTransmission(0x68);                                                                                                                         // inicio comunicacion con MPU6050
  Wire.write(0x3B);                                                                                                                                     // nos vamos al registro 0x3B (acX)
  Wire.endTransmission(false);                                                                                                                          // continuamos la comunicacion I2C
  Wire.requestFrom(0x68, 6, true);                                                                                                                      // solicito los primeros 6 bytes (registros) desde esa direccion, el true indica que luego de recibir los datos se envia un stop

  // Cada valor necesita de 2 registros, ya que justamente pedimos 6 registros de 8 bits porque los valores ocupan 16 bits
  Acc_rawX = Wire.read() << 8 | Wire.read();                                                                                                            // obtengo la aceleracion en X
  Acc_rawY = Wire.read() << 8 | Wire.read();                                                                                                            // obtengo la aceleracion en Y
  Acc_rawZ = Wire.read() << 8 | Wire.read();                                                                                                            // obtengo la aceleracion en Z

  // Calculamos los angulos usando la ecuacion de Euler
  // Para ello a los valores brutos (Acc_raw) la hoja de datos nos recomienda dividirlos por 16384
  // Luego aplicando la formula de Euler hallamos el angulo en radianes, por lo que lo pasamos a sexagesimales
  acelerometro[0] = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * 180 / pi;                          // x
  acelerometro[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * 180 / pi;                     // y

  // Obtenemos los valores del giroscopio (0x43)  
  Wire.beginTransmission(0x68);                                                                                                                         // inicio comunicacion con MPU6050
  Wire.write(0x43);                                                                                                                                     // nos vamos a la direccion 0x43
  Wire.endTransmission(false);                                                                                                                          // continuamos la comunicacion I2C
  Wire.requestFrom(0x68, 4, true);                                                                                                                      // solicito los primeros 4 bytes desde el registro 0x43, esta vez solo 4 porque no usaremos yaw

  Gyr_rawX = Wire.read() << 8 | Wire.read();                                                                                                            // obtengo el giro en X
  Gyr_rawY = Wire.read() << 8 | Wire.read();                                                                                                            // obtengo el giro en Y


  //El datasheet nos recomienda dividir cada valor en 131 para obtener grados por segundo
  giroscopio[0] = Gyr_rawX / 131.0;                                                                                                                     // x
  giroscopio[1] = Gyr_rawY / 131.0;                                                                                                                     // y

  // Aplicamos un filtro para obtener lecturas correctas de los angulos
  angulos[0] = 0.98 * (angulos[0] + giroscopio[0] * tiempoBucle) + 0.02 * acelerometro[0];                                                              // angulo x (Pitch)
  angulos[1] = 0.98 * (angulos[1] + giroscopio[1] * tiempoBucle) + 0.02 * acelerometro[1];                                                              // angulo y (Roll)
  Serial.println(angulos[0]);                                                                                                                           // imprimo el angulo leido
  
  // Ahora comencemos con el diseño del controlador PID con en angulo Y como referencia
  error = angulos[0] - anguloDeseado;                                                                                                                    // calculo el error
  pid_p = kp * error;                                                                                                                                    // creamos el valor proporcional

  // La parte integral se activa para corregir el valor en estado estable por ello se activa de -3 a 3
  if (-3 < error < 3){
    pid_i = pid_i + (ki * error);                                                                                                                        // creamos el valor integral
  }

  // La parte derivada seria el error menos el error anterior
  pid_d = kd * ((error - previous_error) / tiempoBucle);                                                                                                 // creamos el valor derivativo

  // Valor final del PID
  PID = pid_p + pid_i + pid_d;

  // Limitamos el valor del PID ya que los motores solo pueden recibir valores desde 1000 a 2000us
  if (PID < -1000){
    PID = -1000;
  }
  if (PID > 1000){
    PID = 1000;
  }

  // Calculo el valor PWM de los motores corrigiendo su valor con el PID
  pwmLeft = velocidadBase + PID;
  pwmRight = velocidadBase - PID;

  //Limitamos los valores de los motores entre 1000 y 2000
  if (pwmRight < 1000){
    pwmRight = 1000;
  }
  if (pwmRight > 2000){
    pwmRight = 2000;
  }
  if (pwmLeft < 1000){
    pwmLeft = 1000;
  }
  if (pwmLeft > 2000){
    pwmLeft = 2000;
  }

  // Finalmente escribo los valores en los motores
  left_prop.writeMicroseconds(pwmLeft);
  right_prop.writeMicroseconds(pwmRight);
  previous_error = error;
}