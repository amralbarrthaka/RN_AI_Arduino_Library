#include "RN_AI.h"
#include <Arduino.h>
#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "HUSKYLENS.h"

// Error codes
#define ERROR_NONE 0
#define ERROR_INVALID_SPEED 1
#define ERROR_INVALID_DIRECTION 2
#define ERROR_OUT_OF_RANGE 3
#define ERROR_NO_ECHO 4

// Default constructor
RN_AI::RN_AI() : _motorPin1(-1), _motorPin2(-1), _speedPin1(-1), _motorPin3(-1), _motorPin4(-1), _speedPin2(-1), 
           _errorState(false), _errorCode(ERROR_NONE), _movementSpeed(100), _servoInitialized(false),
           _isForward(false), _isBackward(false), _isRotating(false){
    currentDirection = "stop";
    previousDirection = "stop";
    steeringServoInitialized = false;
    cameraServoInitialized = false;
    servoSpeed = 255;
    currentsteeringAngle = 90;
    currentLowAngle = 90;
}

// Constructor Implementations
RN_AI::RN_AI(int motorPin1, int motorPin2, int speedPin1) 
    : _motorPin1(motorPin1), _motorPin2(motorPin2), _speedPin1(speedPin1), _motorPin3(-1), _motorPin4(-1), _speedPin2(-1), 
      _errorState(false), _errorCode(ERROR_NONE) {
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    pinMode(_speedPin1, OUTPUT);
    currentDirection = "stop";
    previousDirection = "stop";
}

// Distance sensor constructor
RN_AI::RN_AI(int trigPin, int echoPin) 
    : _trigPin(trigPin), _echoPin(echoPin), _errorState(false), _errorCode(ERROR_NONE) {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    currentDirection = "stop";
    previousDirection = "stop";
}

// Color sensor constructor
RN_AI::RN_AI(int s0, int s1, int s2, int s3, int out) 
    : _s0(s0), _s1(s1), _s2(s2), _s3(s3), _sensorOut(out), _errorState(false), _errorCode(ERROR_NONE), _servoInitialized(false) {
    pinMode(_s0, OUTPUT);
    pinMode(_s1, OUTPUT);
    pinMode(_s2, OUTPUT);
    pinMode(_s3, OUTPUT);
    pinMode(_sensorOut, INPUT);
    currentDirection = "stop";
    previousDirection = "stop";
}

// L293D shield constructor
RN_AI::RN_AI(int motorPin1, int motorPin2, int speedPin1, int motorPin3, int motorPin4, int speedPin2) 
    : _motorPin1(motorPin1), _motorPin2(motorPin2), _speedPin1(speedPin1), _motorPin3(motorPin3), _motorPin4(motorPin4), _speedPin2(speedPin2), 
      _errorState(false), _errorCode(ERROR_NONE) {
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    pinMode(_speedPin1, OUTPUT);
    pinMode(_motorPin3, OUTPUT);
    pinMode(_motorPin4, OUTPUT);
    pinMode(_speedPin2, OUTPUT);
    currentDirection = "stop";
    previousDirection = "stop";
}

void RN_AI::stopAllMotors() {
    for (int i = 1; i <= 4; i++) {
        stopMotor(i);
    }
}

// Distance sensor methods
float RN_AI::getDistance() {
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    long duration = pulseIn(_echoPin, HIGH);
    if (duration == 0) {
        _errorState = true;
        _errorCode = ERROR_NO_ECHO;
        return -1;
    }
    float distance = duration * 0.034 / 2;
    return distance;
}

float RN_AI::measureDistance() {
    return getDistance();
}

// Color sensor methods
void RN_AI::beginColorSensor() {
    digitalWrite(_s0, HIGH);
    digitalWrite(_s1, LOW);
}

unsigned int RN_AI::getIntensityR() {
    digitalWrite(_s2, LOW);
    digitalWrite(_s3, LOW);
    return pulseIn(_sensorOut, LOW);
}

unsigned int RN_AI::getIntensityG() {
    digitalWrite(_s2, HIGH);
    digitalWrite(_s3, HIGH);
    return pulseIn(_sensorOut, LOW);
}

unsigned int RN_AI::getIntensityB() {
    digitalWrite(_s2, LOW);
    digitalWrite(_s3, HIGH);
    return pulseIn(_sensorOut, LOW);
}

// LED control methods
void RN_AI::setLED(bool state) {
    if (_isLEDControl) {
        digitalWrite(_ledPin, state ? HIGH : LOW);
    }
}

bool RN_AI::controlLED(bool state) {
    if (_isLEDControl) {
        digitalWrite(_ledPin, state ? HIGH : LOW);
        return true;
    }
    return false;
}

// MPU6050 methods
void RN_AI::initializeGyro() {
    Serial.println("Initializing MPU6050...");
    // Initialize MPU6050
    if (!_mpu.begin()) {
        Serial.println("Failed to initialize MPU6050");
        return;
    }
    
    // Configure MPU6050 settings
    _mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // Wait for sensor to stabilize
    delay(1000);
    
    // Initialize timing and target values
    _lastMicros = micros();
    
    Serial.println("MPU6050 initialized successfully");
    correctGyro();
    updateGyro();
    _targetYaw = _yaw;  // Set target to current heading
    Serial.print("Initial heading set to: "); Serial.println(_targetYaw);
}

void RN_AI::correctGyro() {
    Serial.println("Correcting gyroscope (keep sensor still)...");
    float sum = 0.0;
    const int samples = 500;  // Increased samples for better calibration

    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        _mpu.getEvent(&a, &g, &temp);
        sum += g.gyro.z;
        delay(5);
    }

    _correctionValueGyro = sum/samples;
    Serial.print("Gyro Correction Value : ");
    Serial.println(_correctionValueGyro, 6);
    delay(1000);
}

void RN_AI::updateGyro() {
    sensors_event_t a, g, temp;
    _mpu.getEvent(&a, &g, &temp);

    unsigned long currentMicros = micros();
    float deltaTime = (currentMicros - _lastMicros) / 1000000.0; // convert to seconds
    _lastMicros = currentMicros;

    // Get gyro reading and apply drift compensation
    float gyroZ = g.gyro.z - _correctionValueGyro;
    
    // Integrate gyro.z to estimate yaw
    _yaw += gyroZ * (180.0 / PI) * deltaTime;

    // Normalize yaw to [0, 360)
    if (_yaw >= 360.0) _yaw -= 360.0;
    if (_yaw < 0.0) _yaw += 360.0;
}

void RN_AI::setGyroRange(mpu6050_gyro_range_t range) {
    _mpu.setGyroRange(range);
}

void RN_AI::setFilterBandwidth(mpu6050_bandwidth_t bandwidth) {
    _mpu.setFilterBandwidth(bandwidth);
}

// Miscellaneous methods
int RN_AI::getReading() {
    if (_isDigital) {
        return digitalRead(_pin);
    }
    return analogRead(_pin);
}

bool RN_AI::isError() {
    return _errorState;
}

int RN_AI::getErrorCode() {
    return _errorCode;
}

// New motor control methods implementation
bool RN_AI::begin() {


    
    return true;
}

void RN_AI::initializeMotor(int pin1, int pin2, int speedPin) {
    _motorPin1 = pin1;
    _motorPin2 = pin2;
    _speedPin1 = speedPin;
    
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    pinMode(_speedPin1, OUTPUT);
}

void RN_AI::setMovementSpeed(float speed) {
    _movementSpeed = speed;
}

void RN_AI::moveForward() {
    _isForward = true;
    _isBackward = false;
    _isRotating = false;
    currentDirection = "forward";
    digitalWrite(_motorPin1, HIGH);
    digitalWrite(_motorPin2, LOW);
    analogWrite(_speedPin1, _movementSpeed);
}

void RN_AI::moveBackward() {
    // Motor 1 backward
    _isForward = false;
    _isBackward = true;
    _isRotating = false;
    currentDirection = "backward";
    digitalWrite(_motorPin1, LOW);
    digitalWrite(_motorPin2, HIGH);
    analogWrite(_speedPin1, _movementSpeed);
    // Serial.print("Moving backward");
    // Serial.println(" Speed: "); Serial.println(_movementSpeed);
    

}

void RN_AI::stopMovement() {
    _isForward = false;
    _isBackward = false;
    _isRotating = true;
    currentDirection = "stop";
    // Serial.print("Stopping movement");
    // Serial.println(currentDirection);
    // Stop motor 1
    digitalWrite(_motorPin1, LOW);
    digitalWrite(_motorPin2, LOW);
    analogWrite(_speedPin1, 0);
    // Serial.println("Moving stopped");
    
}


void RN_AI::setServoSpeed(int speed) {
    servoSpeed = constrain(speed, 0, 255);
    Serial.print("Servo speed set to: ");
    Serial.println(servoSpeed);
}


void RN_AI::setSteeringServoAngle(int angle) {
    if (!steeringServoInitialized) {
        Serial.println("Steering servo not initialized!");
        return;
    }
    
    // Constrain angle to valid range
    angle = constrain(angle, steeringServoMin, steeringServoMax);
    
    // Calculate delay based on speed (faster speed = shorter delay)
    int delayTime = map(servoSpeed, 0, 255, 50, 5);  // 50ms at slowest, 5ms at fastest
    
    // Move servo gradually to target angle
    while (currentsteeringAngle != angle) {
        if (currentsteeringAngle < angle) {
            currentsteeringAngle++;
        } else {
            currentsteeringAngle--;
        }
        steeringServo.write(currentsteeringAngle);
        delay(delayTime);
    }
}

void RN_AI::resetServos() {
    if (_servoInitialized) {
        Serial.println("Resetting servo to default position");
        setSteeringServoAngle(_servoDefaultPosition);
    } else {
        Serial.println("Error: Cannot reset - Servo not initialized");
    }
}

bool RN_AI::calibrateServo(int minPosition, int maxPosition) {
    if (!_servoInitialized) {
        _errorState = true;
        _errorCode = ERROR_OUT_OF_RANGE;
        Serial.println("Error: Cannot calibrate - Servo not initialized");
        return false;
    }
    
    _servoMinPosition = constrain(minPosition, 0, 180);
    _servoMaxPosition = constrain(maxPosition, 0, 180);
    
    if (_servoMinPosition > _servoMaxPosition) {
        int temp = _servoMinPosition;
        _servoMinPosition = _servoMaxPosition;
        _servoMaxPosition = temp;
    }
    
    Serial.println("Servo calibrated:");
    Serial.print("New Min Position: "); Serial.println(_servoMinPosition);
    Serial.print("New Max Position: "); Serial.println(_servoMaxPosition);
    
    return true;
}

// Gyro movement methods
void RN_AI::adjustDirectionWithGyro() {
    updateGyro();
    // Get current yaw
    float currentYaw = _yaw;
    if (currentDirection != previousDirection) {
        // Reset target yaw when direction changes
        _targetYaw = _yaw;
        previousDirection = currentDirection;
        Serial.print("Changed Direction: "); Serial.println(currentDirection);
    }
    
    
    // Calculate angle error and normalize to [-180, 180]
    float angleError =  _targetYaw - currentYaw;
    if (angleError > 180) angleError -= 360;
    if (angleError < -180) angleError += 360;

    // Calculate proportional correction
    float correction = 0.01 * angleError;
    Serial.print("Correction: "); Serial.println(correction);
    
    // Reverse correction when moving backward
    if (_isBackward) {
        correction = -correction;  // Reverse the correction for backward movement
    }
    // Map correction to servo angle range
    int servoAngle = 90 + correction;
    
    // Constrain servo angle to valid range
    servoAngle = constrain(servoAngle, 45, 135);
    

    setSteeringServoAngle(servoAngle);
    Serial.print("Current Yaw: "); Serial.print(currentYaw);
    Serial.print(" Target Yaw: "); Serial.print(_targetYaw);
    Serial.print(" Correction: "); Serial.print(correction);
    Serial.print(" Servo Angle: "); Serial.println(servoAngle);
    
    // Debug output
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
}

void RN_AI::moveForwardWithGyro() {
    moveForward();
    adjustDirectionWithGyro();

}

void RN_AI::moveBackwardWithGyro() {
    moveBackward();
    adjustDirectionWithGyro();

}


void RN_AI::initializeSteeringServo(int pin, int defaultAngle, int minAngle, int maxAngle) {
    // Attach steering servo to its pin
    steeringServo.attach(pin);
    
    // Store angle limits
    steeringServoMin = constrain(minAngle, 0, 180);
    steeringServoMax = constrain(maxAngle, 0, 180);
    
    // Ensure min is less than max
    if (steeringServoMin > steeringServoMax) {
        int temp = steeringServoMin;
        steeringServoMin = steeringServoMax;
        steeringServoMax = temp;
    }
    
    // Store and set default angle
    steeringServoDefault = constrain(defaultAngle, steeringServoMin, steeringServoMax);
    currentsteeringAngle = steeringServoDefault;
    steeringServo.write(steeringServoDefault);
    
    steeringServoInitialized = true;
    Serial.print("steering servo initialized - Pin: ");
    Serial.print(pin);
    Serial.print(", Default: ");
    Serial.print(steeringServoDefault);
    Serial.print(", Range: ");
    Serial.print(steeringServoMin);
    Serial.print("-");
    Serial.println(steeringServoMax);
}


