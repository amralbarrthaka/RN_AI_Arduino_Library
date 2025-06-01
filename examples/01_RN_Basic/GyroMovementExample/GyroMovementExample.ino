#include <RN_AI.h>

// Define motor pins
#define MOTOR_PIN1 3  // Motor direction pin 1
#define MOTOR_PIN2 4  // Motor direction pin 2
#define MOTOR_SPEED_PIN 5  // Motor speed (PWM) pin
#define MOTOR_SPEED 150 // 0 to 255

// Define steering servo pin
#define STEERING_SERVO_PIN 2  // Pin for steering servo
#define STEERING_CENTER 90    // Center position of steering servo
#define STEERING_RANGE 30     // Range of steering movement

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
    static unsigned long lastStateChange = 0;
    static int state = 0;  // 0: forward, 1: stop1, 2: backward, 3: stop2
    unsigned long currentTime = millis();

    switch (state) {
        case 0:  // Forward
            if (currentTime - lastStateChange >= 3000) {  // After 3 seconds
                robot.stopMovementWithGyro();
                state = 1;
                lastStateChange = currentTime;
            } else {
                robot.moveForwardWithGyro();
            }
            break;
            
        case 1:  // First stop
            if (currentTime - lastStateChange >= 1000) {  // After 1 second
                state = 2;
                lastStateChange = currentTime;
            }
            break;
            
        case 2:  // Backward
            if (currentTime - lastStateChange >= 3000) {  // After 3 seconds
                robot.stopMovementWithGyro();
                state = 3;
                lastStateChange = currentTime;
            } else {
                robot.moveBackwardWithGyro();
            }
            break;
            
        case 3:  // Second stop
            if (currentTime - lastStateChange >= 1000) {  // After 1 second
                state = 0;  // Reset to forward
                lastStateChange = currentTime;
            }
            break;
    }


}