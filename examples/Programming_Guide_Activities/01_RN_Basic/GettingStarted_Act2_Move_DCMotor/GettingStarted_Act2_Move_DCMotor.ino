#include "RN.h"

// Define motor pins
#define MOTOR1_PIN1 3 // Motor direction pin 1
#define MOTOR1_PIN2 4  // Motor direction pin 2
#define MOTOR1_SPEED_PIN 5  // Motor speed (PWM) pin

// Create RN instance for motor control
RN motorController(MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_SPEED_PIN);

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(9600);

    // Move motor forward at full speed
    motorController.setMotorDirection(1, FORWARD); // Motor 1 moving forward
    motorController.setMotorSpeed(1, 255); // Full speed
    delay(1000); // Run forward for 1 second

    // Stop motor
    motorController.stopMotor(1); 
    Serial.println("Motor stopped.");
    delay(1000); // Wait for 1 second

    // Move motor backward at full speed
    motorController.setMotorDirection(1, BACKWARD); // Motor 1 moving backward
    motorController.setMotorSpeed(1, 255); // Full speed
    delay(1000); // Run backward for 1 second

    // Stop motor
    motorController.stopMotor(1); 
    Serial.println("Motor stopped.");
    delay(1000); // Wait for 1 second
}

void loop() {
    // You can add more control code here if needed
}
