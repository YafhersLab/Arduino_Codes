#include <Stepper.h>

#define STEPS 200
Stepper stepper(STEPS, 8,9,10,11);

void setup(){
  stepper.setSpeed(180);
}

void loop(){
  stepper.step(200);
  delay(1000);

  stepper.step(-200);
  delay(1000);
}