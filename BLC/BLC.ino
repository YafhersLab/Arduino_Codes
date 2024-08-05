// ########################################  INCLUDES   #############################################//
#include <Arduino.h>
#include <Wire.h>

// ########################################  DEFINES   #############################################//
#define pi 3.141592654

// ########################################  VARIABLES   #############################################//
//Variables de los motores nema
int step1 = 4;
int dir1 = 3;
int step2 = 6;
int dir2 = 5;

// En el datasheet del MPU6050, la IMU nos entrega 3 valores de 16 bits para la acelaración y 3 para el giro
// estos valores nos lo entrega en 8 bits por lo que tendremos que unirnos para guardar todo en una sola variable
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float acelerometro[3];
float giroscopio[3];
float angulos[3];

// Variables de temporizacion
float tiempoBucle, time, tiempoAnterior;

// Variables del PID
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
double kp = 370;                                                                                                                                         
double ki = 10;                                                                                                                                          
double kd = 500;                                                                                                                                          
float anguloDeseado = -5.665;
int frecuencia;
int i = 2;
int s = 1;


// ########################################  SETUP   #############################################//
void setup() {
  // Primera comunicacion con el MPU6050
  Wire.begin();                                                                                                                                         // iniciar comunicacion serial I2C
  Wire.beginTransmission(0x68);                                                                                                                         // nos conectamos al 0x68 (MPU6050)
  Wire.write(0x6B);                                                                                                                                     // nos vamos al registro 0x6B
  Wire.write(0);                                                                                                                                        // seteo el bit 1, indico que se va a usar el reloj interno del Arduino a 8Mhz
  Wire.endTransmission(true);                                                                                                                           // finaliza la comunicacion I2C
  Wire.beginTransmission(0x68);                                                                                                                         // Iniciar comunicación I2C con la direccion del MPU
  Wire.write(0x1A);                                                                                                                                     // nos vamos al registro 0x1A                      
  Wire.write(0x03);                                                                                                                                     // configura la frecuencia de transmision                      
  Wire.endTransmission(true);                                                                                                                           // Finalizar la comunicación I2C
  
  //Configuración del Timer2
  pinMode(13, OUTPUT);
  TCCR1A = 0;                                                                                                                                           // limpiamos el registro de control A del timer 1
  TCCR1B = 0;                                                                                                                                           // limpiamos el registro de control B del timer 1
  TCCR1B |= B00000011;                                                                                                                                  // prescaler a 64
  TIMSK1 |= B00000010;                                                                                                                                  // habilito la interrupcion por comparacion

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
  // Obtengo el tiempo transcurrido desde el bucle anterior                                                                                                                              // almaceno el tiempo anterior
  time = millis();   

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
  acelerometro[2] = atan(sqrt(pow(Acc_rawX / 16384.0,2) + pow(Acc_rawY / 16384.0, 2)) / (Acc_rawZ / 16384.0 )) * 180 / pi;

  // Obtenemos los valores del giroscopio (0x43)  
  Wire.beginTransmission(0x68);                                                                                                                         // inicio comunicacion con MPU6050
  Wire.write(0x43);                                                                                                                                     // nos vamos a la direccion 0x43
  Wire.endTransmission(false);                                                                                                                          // continuamos la comunicacion I2C
  Wire.requestFrom(0x68, 2, true);                                                                                                                      // solicito los primeros 4 bytes desde el registro 0x43, esta vez solo 4 porque no usaremos yaw

  Gyr_rawX = Wire.read() << 8 | Wire.read();                                                                                                            // obtengo el giro en X
  //Gyr_rawY = Wire.read() << 8 | Wire.read();  
  //Gyr_rawZ = Wire.read() << 8 | Wire.read();                                                                                                          // obtengo el giro en Y
  tiempoBucle = (time - tiempoAnterior) / 1000;                                                                                                         // obtengo el tiempo transcurrido en segundos

  // El datasheet nos recomienda dividir cada valor en 131 para obtener grados por segundo
  giroscopio[0] = Gyr_rawX / 131.0;                                                                                                                     // x
  //giroscopio[1] = Gyr_rawY / 131.0;                                                                                                                     // y
  //giroscopio[2] = Gyr_rawZ / 131.0;

  // Aplicamos un filtro para obtener lecturas correctas de los angulos
  angulos[0] = 0.98 * (angulos[0] + giroscopio[0] * tiempoBucle) + 0.02 * acelerometro[0];                                                              // angulo x (Pitch)
  //angulos[1] = 0.98 * (angulos[1] + giroscopio[1] * tiempoBucle) + 0.02 * acelerometro[1];                                                              // angulo y (Roll)
  //angulos[2] = 0.98 * (angulos[2] + giroscopio[2] * tiempoBucle) + 0.02 * acelerometro[2];                                                              // angulo y (Roll)

  //String imprimeme = String(angulos[0]) + "\t " + String(angulos[1]) + "\t " + String(angulos[2]);
  String imprimeme = "Inclinación: " + String(angulos[0]) + "\t PID valor: " + PID + " \t frecuencia: " + frecuencia + "\t " + (time - tiempoAnterior) + "\t" + String(error);
  tiempoAnterior = time;  
  Serial.println(imprimeme);                                                                                                                           // imprimo el angulo leido

  // Ahora comencemos con el diseño del controlador PID con en angulo Y como referencia
  error = abs(angulos[0] - anguloDeseado);                                                                                                             // calculo el error
  pid_p = kp * error;                                                                                                                                  // creamos el valor proporcional

  // La parte integral se activa para corregir el valor en estado estable por ello se activa de -3 a 3
  if (abs(error)<6){
    pid_i = s*pid_i + (ki * error);
    Serial.println(pid_i);
    if (abs(error)<=1.5){
      s = -0.05;
    }
    if (abs(error)>=2.2){
      s = 0.05;
    }                                                                                                        // creamos el valor integral
  }

  // La parte derivada seria el error menos el error anterior
  pid_d = kd * ((error - previous_error));                                                                                               // creamos el valor derivativo

  // Valor final del PID
  if (abs(angulos[0]-anguloDeseado)<8){
    pid_d = 0;
  }
  PID = pid_p + pid_i + pid_d;

  // Limitamos el valor del PID ya que los motores solo pueden recibir valores desde 1000 a 2000us
  frecuencia = constrain(5000 - PID, 180, 5000);
  OCR1A = frecuencia;
  // i+=1;
  // if (i==1||i==2){
  //   OCR1A = frecuencia;
  // }
  // else{
  //   frecuencia = constrain(1500 - PID, 140, 1500);
  //   OCR1A = frecuencia;
  //   if(i==4){
  //     i=0;
  //   }
  // }
  previous_error = error;

}

ISR(TIMER1_COMPA_vect){
  TCNT1 = 0;         
  // Correcion de direccion
  if (angulos[0] >= anguloDeseado){
    digitalWrite(dir1, LOW);   digitalWrite(dir2, HIGH);
  }
  if(angulos[0] < anguloDeseado){
    digitalWrite(dir1, HIGH);   digitalWrite(dir2, LOW);
  }

  // Generacion de pasos       
  digitalWrite(step1, !(digitalRead(step1)));   digitalWrite(step2, !(digitalRead(step2)));
}