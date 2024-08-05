#include <QTRSensors.h>
#define AIN1 17
#define AIN2 16
#define BIN1 18
#define BIN2 19
#define STBY 15
#define PWMB 21
#define PWMA 4
#define LED1 2
#define LED2 15
#define BTN1 22
#define BTN2 23
//#define LEDON 13
#define LEDON 34
#define S0 12
#define S1 14
#define S2 27
#define S3 26
#define S4 25
#define S5 33
#define S6 32
#define S7 35

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  // Configuracion del GPIO
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(LEDON, OUTPUT);
  pinMode(BTN1, INPUT);
  pinMode(BTN2, INPUT);
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(S6, INPUT);
  pinMode(S7, INPUT);

  // Configuracion Serial
  Serial.begin(115200);

  // Configuracion del QTR8A

}

void loop() {

}

void calibration(){

}

void motores(){

}