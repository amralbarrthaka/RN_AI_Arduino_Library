#include "RN.h"

// Define the pin connected to the servo motor
const int servoPin = 2;

// Create an instance of the RN library
RN servoController(servoPin);

void setup() {
  servoController.Servobegin();  // Initialize the servo
  
}

void loop() {
  // Move the servo to 0 degrees
  servoController.setServoPosition(0);  // Move to 0 degrees
  delay(1000);  // Wait for 1 second

  // Move the servo to 90 degrees
  servoController.setServoPosition(90);  // Move to 90 degrees
  delay(1000);  // Wait for 1 second
}
