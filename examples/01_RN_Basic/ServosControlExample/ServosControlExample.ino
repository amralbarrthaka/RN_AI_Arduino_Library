#include <RN_AI.h>

// Create robot instance
RN_AI robot;

const int STEERING_SERVO_PIN = 2;  // STEERING guide vane servo pin
const int STEERING_SERVO_DEFAULT = 90;  // Center position
const int STEERING_SERVO_MIN = 45;      // Minimum angle
const int STEERING_SERVO_MAX = 135;     // Maximum angle

// Servo speed control
const int SERVO_SPEED = 220;    // Servo speed 0 to 255

void setup() {
    Serial.begin(115200);
    Serial.println("Servos Guide Vanes Example");

    // Set servo speed BEFORE initializing servos
    robot.setServoSpeed(SERVO_SPEED);
    
    // Initialize servos with all parameters
    robot.initializeSteeringServo(STEERING_SERVO_PIN, STEERING_SERVO_DEFAULT, STEERING_SERVO_MIN, STEERING_SERVO_MAX);
    delay(500);  // Short delay between initializations
}

void loop() {
    // Test STEERING servo
    Serial.println("Moving servo to minimum position");
    robot.setSteeringServoAngle(STEERING_SERVO_MIN);
    delay(1000);
    
    Serial.println("Moving servo to center position");
    robot.setSteeringServoAngle(STEERING_SERVO_DEFAULT);
    delay(1000);
    
    Serial.println("Moving servo to maximum position");
    robot.setSteeringServoAngle(STEERING_SERVO_MAX);
    delay(1000);
    
    Serial.println("Moving servo to center position");
    robot.setSteeringServoAngle(STEERING_SERVO_DEFAULT);
    delay(1000);
}
