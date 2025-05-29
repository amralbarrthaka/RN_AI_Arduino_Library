#include "RN.h"

// Define motor pins
#define MOTOR1_PIN1 3 // Motor direction pin 1
#define MOTOR1_PIN2 4  // Motor direction pin 2
#define MOTOR1_SPEED_PIN 5  // Motor speed (PWM) pin

// Define the pin connected to the servo motor
const int servoPin = 2;
RN motorController(MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_SPEED_PIN);

// Create an instance of the RN library
RN servoController(servoPin);

void rightOvercoming()
{
  servoController.setServoPosition(90);  // turn right
  motorController.setMotorDirection(1, FORWARD); // Motor 1 moving forward
  motorController.setMotorSpeed(1, 100); // Full speed
  delay(200);  // Wait for 1 second

   servoController.setServoPosition(0);  // turn left
  motorController.setMotorDirection(1, FORWARD); // Motor 1 moving forward
  motorController.setMotorSpeed(1, 100); // Full speed
  delay(200);  // Wait for 1 second

  servoController.setServoPosition(90);  // turn right
  motorController.setMotorDirection(1, FORWARD); // Motor 1 moving forward
  motorController.setMotorSpeed(1, 100); // Full speed
  delay(100);  // Wait for 1 second

  servoController.setServoPosition(45);  // correct position
  delay(200);  // Wait for 1 second

  

}

void setup() {
  servoController.Servobegin();  // Initialize the servo
 rightOvercoming();
 motorController.setMotorDirection(1, FORWARD); // Motor 1 moving forward
  motorController.setMotorSpeed(1, 255); // Full speed
  delay(200);  // Wait for 1 second
}


void loop() {

  //forward
  
  
}
