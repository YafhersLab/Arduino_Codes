#include <AccelStepper.h>

AccelStepper nemasexo1 (1,4,3);
AccelStepper nemasexo2 (1,6,5);

void setup() {
  Serial.begin(9600);
  nemasexo1.setMaxSpeed(900);
  nemasexo2.setMaxSpeed(900);

  nemasexo1.disableOutputs();
  nemasexo2.disableOutputs();

  nemasexo1.setCurrentPosition(0);
  nemasexo2.setCurrentPosition(50);
  // nemasexo1.setSpeed(900);
  // nemasexo2.setSpeed(900);
  // nemasexo1.runSpeed();
  // nemasexo2.runSpeed();
}
void loop() {

  nemasexo1.setAcceleration(1000);
  nemasexo2.setAcceleration(1000);

  //nemasexo1.runToPosition();
  
  nemasexo1.moveTo(50);
  nemasexo2.moveTo(0);
  while (nemasexo1.currentPosition()!=50 || nemasexo2.currentPosition()!=0){
    nemasexo1.run();
    nemasexo2.run();
  }
  delay(500);

  nemasexo1.moveTo(0);
  nemasexo2.moveTo(50);
  while (nemasexo1.currentPosition()!=0 || nemasexo2.currentPosition()!=50){
    nemasexo1.run();
    nemasexo2.run();
  }
  delay(500);
  
}
