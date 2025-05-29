#include"RN.h"

RN sensor(6, 7); // create an instance of RN class with trig pin 14 and echo pin 15
RN motor(3, 4, 5); // create a new instance of the RN class
RN servoController(2);


void setup() {

Serial.begin(9600);
servoController.Servobegin();  // Initialize the servo
servoController.setServoPosition(90);  // Move to 90 degrees


}

void loop() {
  float distance = sensor.getDistance();
  
  if (distance < 30 ) {

    motor.stopMotor(1); // stop the motor
    servoController.setServoPosition(135);  // Move to 90 degrees
    motor.setMotorDirection(1,1); // rotate the motor in one direction
    motor.setMotorSpeed(1,128); // set the motor speed to maximum
    delay(1000);






  } else{
  servoController.setServoPosition(90);  // Move to 90 degrees
  motor.setMotorDirection(1,1); // rotate the motor in one direction
  motor.setMotorSpeed(1,128); // set the motor speed to maximum


  }

}
