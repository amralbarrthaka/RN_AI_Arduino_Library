#include "RN.h"

// Define motor pins
#define MOTOR_PIN1 3  // Motor direction pin 1
#define MOTOR_PIN2 4  // Motor direction pin 2
#define MOTOR_SPEED_PIN 5  // Motor speed (PWM) pin
#define MOTOR_SPEED 255 // 0 to 255
// Create RN instance for motor control
RN robot;

void setup() {
        // Initialize Serial communication
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("Basic Movement Example");
    
    // Initialize the robot and MPU6050
    if (!robot.begin()) {
        Serial.println("Failed to initialize RN_Sport!");
        while (1) {
            delay(10);
        }
    }

    robot.initializeMotor(MOTOR_PIN1, MOTOR_PIN2, MOTOR_SPEED_PIN);
    robot.setMovementSpeed(MOTOR_SPEED);


}

void loop() {
    // You can add more control code here if needed
    robot.moveForward();
    delay(3000);
    robot.stopMovement();
    delay(3000);
    robot.moveBackward();
    delay(3000);
    robot.stopMovement();
    delay(3000);
}
