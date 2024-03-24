#include <AccelStepper.h>

#define stepPin 33
#define dirPin 25


AccelStepper stepper1(1,stepPin, dirPin);

void setup() {
  // put your setup code here, to run once:
  stepper1.setMaxSpeed(200); // stepper1.setMaxSpeed(200) // if the stepper driver is in full step mode this means the max is 200 steps per second.
  stepper1.setAcceleration(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  stepper1.runToNewPosition(0);
  stepper1.runToNewPosition(200);
  stepper1.runToNewPosition(100);
}
