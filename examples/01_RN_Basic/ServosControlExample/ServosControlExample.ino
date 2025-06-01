#include <RN_AI.h>

// Create robot instance
RN_AI robot;

// Servo pins
const int MOVING_SERVO_PIN = 2;  // 

// Servo configuration
const int MOVING_SERVO_DEFAULT = 90;  // Center position
const int MOVING_SERVO_MIN = 60;      // Minimum angle
const int MOVING_SERVO_MAX = 120;      // Maximum angle

// Servo speed control
const int SERVO_SPEED = 255;    // Servo speed 0 to 255

void setup() {
    Serial.begin(115200);
    Serial.println("Servos Guide Vanes Example");

    // Set servo speed BEFORE initializing servos
    robot.setServoSpeed(SERVO_SPEED);
    
    // Initialize servos with all parameters
    robot.initializeServo(MOVING_SERVO_PIN, MOVING_SERVO_DEFAULT, MOVING_SERVO_MIN, MOVING_SERVO_MAX);
    delay(500);  // Short delay between initializations
}

void loop() {
    // Test top servo
    Serial.println("Moving servo to minimum position");
    robot.setServoAngle(MOVING_SERVO_MIN);
    delay(1000);
    
    Serial.println("Moving servo to center position");
    robot.setServoAngle(MOVING_SERVO_DEFAULT);
    delay(1000);
    
    Serial.println("Moving servo to maximum position");
    robot.setServoAngle(MOVING_SERVO_MAX);
    delay(1000);
    
    // Reset both servos
    Serial.println("Resetting servos to default positions");
    robot.resetServos();
    delay(2000);
}
