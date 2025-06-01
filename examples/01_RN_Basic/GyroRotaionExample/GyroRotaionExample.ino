#include <RN_AI.h>

// Define motor pins
#define MOTOR_PIN1 3  // Motor direction pin 1
#define MOTOR_PIN2 4  // Motor direction pin 2
#define MOTOR_SPEED_PIN 5  // Motor speed (PWM) pin
#define MOTOR_SPEED 150 // 0 to 255

// Define steering servo pin
#define STEERING_SERVO_PIN 2  // Pin for steering servo
#define STEERING_CENTER 90    // Center position of steering servo
#define STEERING_RANGE 20     // Range of steering movement

// Movement timing constants
#define MOVE_DURATION 2000    // 5 seconds for each movement
#define STOP_DURATION 100    // 5 seconds stop between movements
#define ROTATION_ANGLE 90     // 90 degrees rotation

// Create RN_AI instance
RN_AI robot;

static bool isMoving = true;  // Start in stop state
static unsigned long lastStateChange = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n=== RN_AI Gyro Rotation Example ===");
    
    // Initialize the robot
    if (!robot.begin()) {
        Serial.println("Failed to initialize robot!");
        while (1) delay(10);
    }
    
    // Initialize motor
    robot.initializeMotor(MOTOR_PIN1, MOTOR_PIN2, MOTOR_SPEED_PIN);
    robot.setMovementSpeed(MOTOR_SPEED);
    Serial.println("Motor initialized with speed: " + String(MOTOR_SPEED));
    
    // Initialize steering servo
    robot.initializeSteeringServo(STEERING_SERVO_PIN, STEERING_CENTER, STEERING_RANGE);
    
    // Initialize gyroscope
    robot.initializeGyro();
    
    // Set gyro parameters
    robot.setGyroKp(10.0);  // Reduced Kp for smoother steering
    Serial.println("Gyro initialized with Kp: 10.0");
    
    Serial.println("Initialization complete!");
    Serial.println("=============================\n");
    

    lastStateChange = millis();

}

void loop() {
    unsigned long currentTime = millis();

    if (isMoving) {
        if (currentTime - lastStateChange >= MOVE_DURATION) {
            robot.stopMovementWithGyro();
            
            isMoving = false;
            lastStateChange = currentTime;
            Serial.println("Entering stop state...");
        } else {
            robot.moveForwardWithGyro();
            delay(5);
            robot.stopMovementWithGyro();
            delay(15);
        }
    } else {
        // Stopping state
        if (currentTime - lastStateChange >= STOP_DURATION) {
            isMoving = true;
            lastStateChange = currentTime;
            // Update target angle by adding 90 degrees
            robot.updateTargetAngle(ROTATION_ANGLE);
            Serial.print("New Target Yaw: "); Serial.print(robot.getTargetYaw()); Serial.println("°");
            Serial.println("Starting next movement...");
        }
    }
}