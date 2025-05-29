#include "RN.h"

RN sensor(14, 15); // create an instance of RN class with trig pin 14 and echo pin 15
RN motor(8, 9, 10); // create a new instance of the RN class

void setup() {
  //do nothing
  
}

void loop() {
  
  
  motor.setMotorDirection(1,1); // rotate the motor in one direction
  motor.setMotorSpeed(1,255); // set the motor speed to maximum
  float distance = sensor.getDistance();
  if(distance<50)
  {
  motor.stopMotor(1); // stop the motor
  }
  
}
