//######################################################    INCLUSIONES     #####################################################//
#include <Arduino.h>
#include <Wire.h>

//######################################################    VARIABLES     #####################################################//
int direccionMPU = 0x68;
int calibracionMPU = -7902;                            //Ingrese el valor de calibración del acelerómetro

//pines de los nema
int stepA = 4;
int dirA = 3;
int stepB = 6;
int dirB = 5;
// Variables del controlador PID
float kp = 38;
float ki = 1;
float kd = 36;
float velocidad_giro = 400;                                    //Velocidad de giro (900)
float velocidad_maxima_objetivo = 1400;                                //Velocidad máxima objetivo (1500)
byte inicio, mensaje, bateria_baja;
int nemaA, acelerador_nemaA, contador_acelerador_nemaA, memoria_acelerador_nemaA;
int nemaB, acelerador_nemaB, contador_acelerador_nemaB, memoria_acelerador_nemaB;
int voltaje_bateria;
int contador_recepcion;
int giroscopio_inclinacion_datos_crudos, giroscopio_yaw_datos_crudos, acelerometro_datos_crudos;
long valor_calibracion_giroscopio_yaw, valor_calibracion_giroscopio_inclinacion;
unsigned long temporizador_bucle;
float angulo_giroscopio, angulo_acelerometro, angulo, punto_ajuste_pid_autoequilibrio;
float error_pid_temporal, memoria_pid_i, punto_ajuste_pid, entrada_giroscopio, salida_pid, ultimo_error_d_pid;
float salida_pid_izquierda, salida_pid_derecha;
int velocidad_m = 1000; //máximo 2500
float recogida = 0.009;

//######################################################    CONFIGURACIÓN     #####################################################//
void setup() {
  //Inicializacion de perifericos
  Serial.begin(9600);
  Wire.begin();

  //Configuracion del Timer 2 para habilitar su interrupcion por comparacion
  TWBR = 12;                                                                          // Configuracion de la tasa de baudios para el I2C
  TCCR2A = 0;                                                                         // Limpia el registro de control A del Timer 2
  TCCR2B = 0;                                                                         // Limpia el registro de control B del Timer 2  
  TIMSK2 |= (1 << OCIE2A);                                                            // Habilitar interrupcion por comparacion de salida del timer 2
  TCCR2B |= (1 << CS21);                                                              // Configurar el prescaler del Timer 2: fclk / 8
  OCR2A = 39;                                                                         // Valor de comparacion A en 39
  TCCR2A |= (1 << WGM21);                                                             // Modo de temporizacion CTC, reiniciar el timer al llegar a A


  Wire.beginTransmission(direccionMPU);                                               // Iniciar comunicación I2C con la direccion del MPU
  Wire.write(0x6B);                                                                   
  Wire.write(0x00);
  Wire.endTransmission();                                                             // Finalizar la comunicación I2C
  Wire.beginTransmission(direccionMPU);                                               // Iniciar comunicación I2C con la direccion del MPU
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();                                                             // Finalizar la comunicación I2C
  Wire.beginTransmission(direccionMPU);                                               // Iniciar comunicación I2C con la direccion del MPU
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();                                                             // Finalizar la comunicación I2C
  Wire.beginTransmission(direccionMPU);                                               // Iniciar comunicación I2C con la direccion del MPU
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();                                                             // Finalizar la comunicación I2C

  // internamente los pines de la CNC están conectados a 2,3,5,6 para paso y dirección respectivamente
  pinMode(stepA, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(stepB, OUTPUT); 
  pinMode(dirB, OUTPUT);


  for (contador_recepcion = 0; contador_recepcion < 500; contador_recepcion++) {     //Crear 500 bucles
    if (contador_recepcion % 15 == 0)digitalWrite(13, !digitalRead(13));       //Cambiar el estado del LED cada 15 bucles para hacer que el LED parpadee rápidamente
    Wire.beginTransmission(direccionMPU);                                   //Iniciar comunicación con el giroscopio
    Wire.write(0x43);                                                       //Comenzar a leer el registro Who_am_I 75h
    Wire.endTransmission();                                                 //Finalizar la transmisión
    Wire.requestFrom(direccionMPU, 4);                                      //Solicitar 2 bytes al giroscopio
    valor_calibracion_giroscopio_yaw += Wire.read() << 8 | Wire.read();           //Combinar los dos bytes para hacer un entero
    valor_calibracion_giroscopio_inclinacion += Wire.read() << 8 | Wire.read();         //Combinar los dos bytes para hacer un entero
    delayMicroseconds(3700);                                                //Esperar 3700 microsegundos para simular el tiempo de bucle del programa principal
  }
  valor_calibracion_giroscopio_inclinacion /= 500;                                      //Dividir el valor total por 500 para obtener la desviación gyro promedio
  valor_calibracion_giroscopio_yaw /= 500;                                        //Dividir el valor total por 500 para obtener la desviación gyro promedio

  temporizador_bucle = micros() + 4000;                                             //Establecer la variable temporizador_bucle en el próximo tiempo de finalización del bucle

}

//######################################################    BUCLE    #####################################################//
void loop() {
  if (Serial.available()) {
    mensaje = Serial.read();
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Cálculos de ángulo
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(direccionMPU);                                     //Iniciar comunicación con el giroscopio
  Wire.write(0x3F);                                                         //Comenzar a leer en el registro 3F
  Wire.endTransmission();                                                   //Finalizar la transmisión
  Wire.requestFrom(direccionMPU, 2);                                        //Solicitar 2 bytes al giroscopio
  acelerometro_datos_crudos = Wire.read() << 8 | Wire.read();                  //Combinar los dos bytes para hacer un entero
  acelerometro_datos_crudos += calibracionMPU;                          //Agregar el valor de calibración del acelerómetro
  if (acelerometro_datos_crudos > 8200) {
    acelerometro_datos_crudos = 8200;          //Prevenir la división por cero limitando los datos del acelerómetro a +/-8200;
  }
  if (acelerometro_datos_crudos < -8200) {
    acelerometro_datos_crudos = -8200;        //Prevenir la división por cero limitando los datos del acelerómetro a +/-8200;
  }

  angulo_acelerometro = asin((float)acelerometro_datos_crudos / 8200.0) * 57.296;        //Calcular el ángulo actual según el acelerómetro

  if (inicio == 0 && angulo_acelerometro > -0.5 && angulo_acelerometro < 0.5) {                  //Si el ángulo del acelerómetro está casi en 0
    angulo_giroscopio = angulo_acelerometro;                                                 //Cargar el ángulo del acelerómetro en la variable angulo_giroscopio
    inicio = 1;                                                              //Establecer la variable de inicio para iniciar el controlador PID
  }

  Wire.beginTransmission(direccionMPU);                                     //Iniciar comunicación con el giroscopio
  Wire.write(0x43);                                                         //Comenzar a leer en el registro 43
  Wire.endTransmission();                                                   //Finalizar la transmisión
  Wire.requestFrom(direccionMPU, 4);                                        //Solicitar 4 bytes al giroscopio
  giroscopio_yaw_datos_crudos = Wire.read() << 8 | Wire.read();                       //Combinar los dos bytes para hacer un entero
  giroscopio_inclinacion_datos_crudos = Wire.read() << 8 | Wire.read();                     //Combinar los dos bytes para hacer un entero

  giroscopio_inclinacion_datos_crudos -= valor_calibracion_giroscopio_inclinacion;                      //Agregar el valor de calibración del giroscopio
  angulo_giroscopio += giroscopio_inclinacion_datos_crudos * 0.000031;                             //Calcular el ángulo recorrido durante este bucle y agregarlo a la variable angulo_giroscopio

  giroscopio_yaw_datos_crudos -= valor_calibracion_giroscopio_yaw;                          //Agregar el valor de calibración del giroscopio
  //  angulo_giroscopio -= giroscopio_yaw_datos_crudos * 0.0000003;                          //Compensar la desviación del giroscopio cuando el robot está rotando

  angulo_giroscopio = angulo_giroscopio * 0.9996 + angulo_acelerometro * 0.0004;                    //Corregir la deriva del ángulo del giroscopio con el ángulo del acelerómetro
  error_pid_temporal = angulo_giroscopio - punto_ajuste_pid_autoequilibrio - punto_ajuste_pid;
  if (salida_pid > 10 || salida_pid < -10) {
    error_pid_temporal += salida_pid * 0.015 ;
  }

  memoria_pid_i += ki * error_pid_temporal;
  if (memoria_pid_i > velocidad_m) {
    memoria_pid_i = velocidad_m;
  }
  else if (memoria_pid_i < -velocidad_m) {
    memoria_pid_i = -velocidad_m;
  }
  //Calcular el valor de salida del PID
  salida_pid = kp * error_pid_temporal + memoria_pid_i + kd * (error_pid_temporal - ultimo_error_d_pid);
  if (salida_pid > velocidad_m) {
    salida_pid = velocidad_m;
  }
  else if (salida_pid < -velocidad_m) {
    salida_pid = -velocidad_m;
  }

  ultimo_error_d_pid = error_pid_temporal;

  if (salida_pid < 5 && salida_pid > -5) {
    salida_pid = 0;
  }

  if (angulo_giroscopio > 40 || angulo_giroscopio < -40 || inicio == 0 || bateria_baja == 1) {
    salida_pid = 0;
    memoria_pid_i = 0;
    inicio = 0;
    punto_ajuste_pid_autoequilibrio = 0;
  }

  salida_pid_izquierda = salida_pid;
  salida_pid_derecha = salida_pid;

  if (mensaje == 82) {
    salida_pid_izquierda += velocidad_giro;
    salida_pid_derecha -= velocidad_giro;
  }
  if (mensaje == 76) {
    salida_pid_izquierda -= velocidad_giro;
    salida_pid_derecha += velocidad_giro;
  }

  if (mensaje == 70) {
    if (punto_ajuste_pid > -2.5) {
      punto_ajuste_pid -= recogida;
    }
    if (salida_pid > velocidad_maxima_objetivo * -1) {
      punto_ajuste_pid -= recogida;
    }
  }
  if (mensaje == 66) {
    if (punto_ajuste_pid < 2.5) {
      punto_ajuste_pid += recogida;
    }
    if (salida_pid < velocidad_maxima_objetivo) {
      punto_ajuste_pid += recogida;
    }
  }

  if (mensaje == 83) {
    if (punto_ajuste_pid > 0.5) {
      punto_ajuste_pid -= recogida;
    }
    else if (punto_ajuste_pid < -0.5) {
      punto_ajuste_pid += recogida;
    }
    else punto_ajuste_pid = 0;
  }
  
  if (punto_ajuste_pid == 0) {
    if (salida_pid < 0) {
      punto_ajuste_pid_autoequilibrio += 0.0015;
    }
    if (salida_pid > 0) {
      punto_ajuste_pid_autoequilibrio -= 0.0015;
    }
  }
  if (salida_pid_izquierda > 0) {
    salida_pid_izquierda = velocidad_m - (1 / (salida_pid_izquierda + 9)) * 5500;
  }
  else if (salida_pid_izquierda < 0) {
    salida_pid_izquierda = -velocidad_m - (1 / (salida_pid_izquierda - 9)) * 5500;
  }

  if (salida_pid_derecha > 0) {
    salida_pid_derecha = velocidad_m - (1 / (salida_pid_derecha + 9)) * 5500;
  }
  else if (salida_pid_derecha < 0) {
    salida_pid_derecha = -velocidad_m - (1 / (salida_pid_derecha - 9)) * 5500;
  }

  //Calcular el tiempo de pulso necesario para los controladores de motores paso a paso izquierdo y derecho
  if (salida_pid_izquierda > 0) {
    nemaA = velocidad_m - salida_pid_izquierda;
  }
  else if (salida_pid_izquierda < 0) {
    nemaA = -velocidad_m - salida_pid_izquierda;
  }
  else {
    nemaA = 0;
  }

  if (salida_pid_derecha > 0) {
    nemaB = velocidad_m - salida_pid_derecha;
  }
  else if (salida_pid_derecha < 0) {
    nemaB = -velocidad_m - salida_pid_derecha;
  }
  else {
    nemaB = 0;
  }
  //Serial.println(nemaA);
  acelerador_nemaA = nemaA;
  acelerador_nemaB = nemaB;
  while (temporizador_bucle > micros());
  temporizador_bucle += 4000;
}


ISR(TIMER2_COMPA_vect) {
  //Cálculos de pulso del motor izquierdo
  contador_acelerador_nemaA ++;
  if (contador_acelerador_nemaA > memoria_acelerador_nemaA) {
    contador_acelerador_nemaA = 0;
    memoria_acelerador_nemaA = acelerador_nemaA;
    if (memoria_acelerador_nemaA < 0) {
      PORTD &= 0b10111111;
      memoria_acelerador_nemaA *= -1;
    }
    else PORTD |= 0b01000000;
  }
  else if (contador_acelerador_nemaA == 1) {
    PORTD |= 0b00001000;
  }
  else if (contador_acelerador_nemaA == 2) {
    PORTD &= 0b11110111;
  }

  //cálculos de pulso del motor derecho
  contador_acelerador_nemaB ++;
  if (contador_acelerador_nemaB > memoria_acelerador_nemaB) {
    contador_acelerador_nemaB = 0;
    memoria_acelerador_nemaB = acelerador_nemaB;
    if (memoria_acelerador_nemaB < 0) {
      PORTD |= 0b00100000;
      memoria_acelerador_nemaB *= -1;
    }
    else PORTD &= 0b11011111;
  }
  else if (contador_acelerador_nemaB == 1) {
    PORTD |= 0b00000100;           //Establecer salida 2 en alto para crear un pulso para el controlador paso a paso
  }
  else if (contador_acelerador_nemaB == 2) {
    PORTD &= 0b11111011;
  }
}
