//######################################################    INCLUDES     #####################################################//
#include <Arduino.h>
#include <BluetoothSerial.h>

//######################################################    DEFINES     #####################################################//
#define motorA1 12
#define motorA2 13
#define motorB1 14
#define motorB2 27
#define pwmA 25
#define pwmB 26
#define servo 5

//######################################################   OBJETOS    ########################################################//
BluetoothSerial SerialBT;

//######################################################   FUNCIONES    ########################################################//
void motores(int velA, int velB);

//######################################################    SETUP     #####################################################//
void setup() {
    //Iniciamos los perifericos
    Serial.begin(9600);
    SerialBT.begin("ESP32test");

    //Configuramos los pines
    pinMode(motorA1, OUTPUT);
    pinMode(motorA2, OUTPUT);
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);

    //Configuramos el PWM
    ledcSetup(2, 5000, 8);
    ledcAttachPin(pwmA, 2);
    ledcSetup(1, 5000, 8);
    ledcAttachPin(pwmB, 1);
}

//######################################################    LOOP     #####################################################//
void loop() {
    if(SerialBT.available()){
        char dato = SerialBT.read();
        if(dato == 'E'){
            motores(50, 0);
        }
        else if(dato == 'F'){
            motores(0, 0);
        }
    }
}

void motores(int velA, int velB ){
    if(velA >= 0){
        ledcWrite(2, velA);
        digitalWrite(motorA1, HIGH);
        digitalWrite(motorA2, LOW);
    }
    else{
        velA *= -1;
        ledcWrite(2, velA);
        digitalWrite(motorA1, LOW);
        digitalWrite(motorA2, HIGH);
    }
    if(velB >= 0){
        ledcWrite(1, velB);
        digitalWrite(motorB1, HIGH);
        digitalWrite(motorB2, LOW);
    }
    else{
        velB *= -1;
        ledcWrite(1, velB);
        digitalWrite(motorB1, LOW);
        digitalWrite(motorB2, HIGH);
    }
}