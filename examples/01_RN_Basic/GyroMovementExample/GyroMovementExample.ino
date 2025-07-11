#include <RN_AI.h>

// Define motor pins
#define MOTOR_PIN1 3  // Motor direction pin 1
#define MOTOR_PIN2 4  // Motor direction pin 2
#define MOTOR_SPEED_PIN 5  // Motor speed (PWM) pin
#define MOTOR_SPEED 255 // 0 to 255

// Define steering servo pin
#define STEERING_SERVO_PIN 2  // Pin for steering servo
#define STEERING_CENTER 98    // Center position of steering servo
#define STEERING_RANGE 45     // Range of steering movement

// Create RN_AI instance
RN_AI robot;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("RN_AI Gyro Movement Example");
    
    // Initialize the robot
    if (!robot.begin()) {
        Serial.println("Failed to initialize robot!");
        while (1) delay(10);
    }
    
    // Initialize motor
    robot.initializeMotor(MOTOR_PIN1, MOTOR_PIN2, MOTOR_SPEED_PIN);
    robot.setMovementSpeed(MOTOR_SPEED);
    
    // Initialize steering servo
    robot.initializeSteeringServo(STEERING_SERVO_PIN, STEERING_CENTER, STEERING_RANGE);
    
    // Initialize gyroscope
    robot.initializeGyro();
    
    // Set initial target yaw and gyro parameters
    robot.setTargetYaw(0.0);
    robot.setGyroKp(2.0);  // Reduced Kp for smoother steering
    
    delay(1000);
}

void loop() {

robot.moveForwardWithGyro();

}