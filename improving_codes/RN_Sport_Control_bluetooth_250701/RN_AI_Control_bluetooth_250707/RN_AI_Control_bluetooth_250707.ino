#include <RN_AI.h>

#include <QueueArray.h>

QueueArray<String> commandQueue(10);

SoftwareSerial Bluetooth(10, 7); // Arduino (RX, TX) - HC-05 Bluetooth (TX, RX)

// Create RN_Sport instance with default motor pins and base speed
RN_AI robot;


// Define motor pins
#define MOTOR_PIN1 3  // Motor direction pin 1
#define MOTOR_PIN2 4  // Motor direction pin 2
#define MOTOR_SPEED_PIN 5  // Motor speed (PWM) pin
#define MOTOR_SPEED 255 // 0 to 255

// Servo pins
const int MOVING_SERVO_PIN = 2;  // 

// Servo configuration
const int MOVING_SERVO_DEFAULT = 98;  // Center position
const int MOVING_SERVO_MIN = 45;      // Minimum angle
const int MOVING_SERVO_MAX = 135;      // Maximum angle

// Servo speed control
const int SERVO_SPEED = 255;    // Servo speed 0 to 255

const int step_angle = 25;

int Speed = 255; // Initial speed
String direction = "d,s";
String last_direction = "d,s";
String mode = "m";
int auto_program = 0;


#define BUFFER_SIZE 10
String commandBuffer[BUFFER_SIZE];
int bufferStart = 0;
int bufferEnd = 0;
bool bufferFull = false;
//String mode = "a"
void enqueueCommand(String command) {
    commandBuffer[bufferEnd] = command;
    bufferEnd = (bufferEnd + 1) % BUFFER_SIZE;
    if (bufferFull) {
        bufferStart = (bufferStart + 1) % BUFFER_SIZE;
    }
    bufferFull = bufferEnd == bufferStart;
}

String dequeueCommand() {
    if (bufferStart == bufferEnd && !bufferFull) {
        return ""; // Buffer is empty
    }
    String command = commandBuffer[bufferStart];
    bufferStart = (bufferStart + 1) % BUFFER_SIZE;
    bufferFull = false;
    return command;
}


void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("RN_AI Gyro Movement Example");
    
    // Initialize the robot
    if (!robot.begin()) {
        Serial.println("Failed to initialize robot!");
        while (1) delay(10);
    }
    Bluetooth.begin(9600);
    // Initialize motor
    robot.initializeMotor(MOTOR_PIN1, MOTOR_PIN2, MOTOR_SPEED_PIN);
    delay(500);  // Short delay between initializations
    robot.setMovementSpeed(MOTOR_SPEED);
    
    
    // Initialize gyroscope
    robot.initializeGyro();
    delay(500);  // Short delay between initializations

    // Set initial target yaw and gyro parameters
    robot.setTargetYaw(0.0);
    robot.setGyroKp(2.0);  // Reduced Kp for smoother steering
    
      // Set servo speed BEFORE initializing servos
    robot.setServoSpeed(SERVO_SPEED);
    
    // Initialize servos with all parameters
    robot.initializeServo(MOVING_SERVO_PIN, MOVING_SERVO_DEFAULT, MOVING_SERVO_MIN, MOVING_SERVO_MAX);
    

    delay(1000);
    Serial.println("RN_AI initialized successfully!");
}

void loop() {
    // Check for available Bluetooth data and add to command buffer
    if (Bluetooth.available()) {
        String command = Bluetooth.readStringUntil('\n');
        Serial.print(command);
        enqueueCommand(command);
    }

    // Process commands from the buffer
    String command = dequeueCommand();
    if (command != "") {
        command.trim();
        Serial.print("Received: ");
        Serial.println(command);

        if (command.startsWith("d,")) {
            direction = command;
            Serial.print("Setting direction to ");
            Serial.println(direction);
        }else if (command.startsWith("mo,")) {
            mode = command.substring(3);
            Serial.print("Setting mode to ");
            Serial.println(mode);
            if(mode == "m"){
              auto_program = 0;
            }
        }else if (command.startsWith("pr,")) {
            auto_program = command.substring(3).toInt();
            Serial.print("Setting program to ");
            Serial.println(auto_program);
        } else if (command.startsWith("s1,")) {
            int speedValue = command.substring(3).toInt();
            if (speedValue >= 0 && speedValue <= 100) {
              Speed = map(speedValue, 0, 100, 0, 255);
              robot.setMovementSpeed(Speed);
            } else {
                Serial.println("Invalid speed value");
            }
        } else if (command.startsWith("s2,")) {
            int speedValue = command.substring(3).toInt();
            if (speedValue >= 0 && speedValue <= 100) {
                // Speed_C = map(speedValue, 0, 100, 0, 255);
                // robot.setKickSpeed(Speed_C);
                // delay(50);
            } else {
                Serial.println("Invalid speed value");
            }
        } else if (command.startsWith("t1,")) {
            int targetValue1 = command.substring(3).toInt();
            if (targetValue1 >= 0 && targetValue1 <= 100) {
                // TOP_SERVO_CurrentAngle = map(targetValue1, 0, 100, TOP_SERVO_MIN, TOP_SERVO_MAX);
                // Serial.print("Setting target1 to ");
                // Serial.println(TOP_SERVO_CurrentAngle);
                // robot.setTopServoAngle(TOP_SERVO_CurrentAngle);
            } else {
                Serial.println("Invalid target1 value");
            }
        } else if (command.startsWith("t2,")) {
            int targetValue2 = command.substring(3).toInt();
            if (targetValue2 >= 0 && targetValue2 <= 100) {
                // LOW_SERVO_CurrentAngle = map(targetValue2, 0, 100, LOW_SERVO_MIN, LOW_SERVO_MAX);
                // Serial.print("Setting target2 to ");
                // Serial.println(LOW_SERVO_CurrentAngle);
                // robot.setLowServoAngle(LOW_SERVO_CurrentAngle);
            } else {
                Serial.println("Invalid target2 value");
            }
        } else if (command.startsWith("k1,")) {
            String commandbtn = command.substring(3);
            if (commandbtn == "k") {
              Serial.println("kick");
            //   robot.kickForward();
            } else {
              Serial.println("stop_kick");
            //   robot.stopKick();
            }
        } else {
            Serial.println("Invalid command");
        }
    }

    if(last_direction != direction){
        last_direction = direction;
        // Existing direction handling code...
        if (direction == "d,f") {
            robot.setServoAngle(MOVING_SERVO_DEFAULT);
            robot.moveForwardWithGyro();
            // Serial.println("Forward");
        } else if (direction == "d,b") {
            robot.setServoAngle(MOVING_SERVO_DEFAULT);
            robot.moveBackwardWithGyro();                                            
            // Serial.println("Backward");
        } else if (direction == "d,l") {
            robot.setServoAngle(MOVING_SERVO_MIN);
            robot.moveForward();
            Serial.println("Left");
        } else if (direction == "d,r") {
            robot.setServoAngle(MOVING_SERVO_MAX);
            robot.moveForward();
            Serial.println("Right");
        } else if (direction == "d,fl") {
            robot.setServoAngle(MOVING_SERVO_DEFAULT - step_angle);
            robot.moveForward();
            Serial.println("fl");
        } else if (direction == "d,fr") {
            robot.setServoAngle(MOVING_SERVO_DEFAULT + step_angle);
            robot.moveForward();
            Serial.println("fr");
        } else if (direction == "d,bl") {
            robot.setServoAngle(MOVING_SERVO_DEFAULT - step_angle);
            robot.moveBackward();
            Serial.println("bl");
        } else if (direction == "d,br") {
            robot.setServoAngle(MOVING_SERVO_DEFAULT + step_angle);
            robot.moveBackward();
        } else if (direction == "d,s") {
            robot.setServoAngle(MOVING_SERVO_DEFAULT);
            robot.stopMovementWithGyro();
        }
    }

}
