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
           _isForward(false), _isBackward(false), _isRotating(false), _gyroKp(10.0),
           _steeringServoPin(-1), _steeringCenter(90), _steeringRange(30) {

}

// Constructor Implementations
RN_AI::RN_AI(int motorPin1, int motorPin2, int speedPin1) 
    : _motorPin1(motorPin1), _motorPin2(motorPin2), _speedPin1(speedPin1), _motorPin3(-1), _motorPin4(-1), _speedPin2(-1), 
      _errorState(false), _errorCode(ERROR_NONE), _servoInitialized(false) {
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    pinMode(_speedPin1, OUTPUT);
}

// Distance sensor constructor
RN_AI::RN_AI(int trigPin, int echoPin) 
    : _trigPin(trigPin), _echoPin(echoPin), _errorState(false), _errorCode(ERROR_NONE), _servoInitialized(false) {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

// Color sensor constructor
RN_AI::RN_AI(int s0, int s1, int s2, int s3, int out) 
    : _s0(s0), _s1(s1), _s2(s2), _s3(s3), _sensorOut(out), _errorState(false), _errorCode(ERROR_NONE), _servoInitialized(false) {
    pinMode(_s0, OUTPUT);
    pinMode(_s1, OUTPUT);
    pinMode(_s2, OUTPUT);
    pinMode(_s3, OUTPUT);
    pinMode(_sensorOut, INPUT);
}

// Servo constructor
RN_AI::RN_AI(int servoPin) 
    : _servoPin(servoPin), _errorState(false), _errorCode(ERROR_NONE), _servoInitialized(false), _servoSpeed(RN_SERVO_DEFAULT_SPEED) {
    initializeServo(servoPin, RN_SERVO_DEFAULT_ANGLE, RN_SERVO_MIN_ANGLE, RN_SERVO_MAX_ANGLE);
}

// LED constructor
RN_AI::RN_AI(int ledPin, bool isLED) 
    : _ledPin(ledPin), _isLEDControl(isLED), _errorState(false), _errorCode(ERROR_NONE), _servoInitialized(false) {
    if (_isLEDControl) {
        pinMode(_ledPin, OUTPUT);
        digitalWrite(_ledPin, LOW);  // Ensure LED is off initially
    }
}

// Digital/Analog reading constructor
RN_AI::RN_AI(int pin, bool isDigital, bool isInput)
    : _pin(pin), _isDigital(isDigital), _isInputMode(isInput), _errorState(false), _errorCode(ERROR_NONE), _servoInitialized(false) {
    pinMode(_pin, isInput ? INPUT : OUTPUT);
}

// L293D shield constructor
RN_AI::RN_AI(int motorPin1, int motorPin2, int speedPin1, int motorPin3, int motorPin4, int speedPin2) 
    : _motorPin1(motorPin1), _motorPin2(motorPin2), _speedPin1(speedPin1), _motorPin3(motorPin3), _motorPin4(motorPin4), _speedPin2(speedPin2), 
      _errorState(false), _errorCode(ERROR_NONE), _servoInitialized(false) {
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    pinMode(_speedPin1, OUTPUT);
    pinMode(_motorPin3, OUTPUT);
    pinMode(_motorPin4, OUTPUT);
    pinMode(_speedPin2, OUTPUT);

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
    // Motor 1 forward
    digitalWrite(_motorPin1, HIGH);
    digitalWrite(_motorPin2, LOW);
    analogWrite(_speedPin1, _movementSpeed);
    

}

void RN_AI::moveBackward() {
    // Motor 1 backward
    digitalWrite(_motorPin1, LOW);
    digitalWrite(_motorPin2, HIGH);
    analogWrite(_speedPin1, _movementSpeed);
    

}

void RN_AI::stopMovement() {
    // Stop motor 1
    digitalWrite(_motorPin1, LOW);
    digitalWrite(_motorPin2, LOW);
    analogWrite(_speedPin1, 0);
    
}

// Servo control methods
void RN_AI::initializeServo(int pin, int defaultPos, int minPos, int maxPos) {
    _servoPin = pin;
    _servoDefaultPosition = constrain(defaultPos, minPos, maxPos);
    _servoMinPosition = minPos;
    _servoMaxPosition = maxPos;
    
    // Don't reset speed here - it should be set by constructor or setServoSpeed
    _servoMotor.attach(_servoPin);
    _servoMotor.write(_servoDefaultPosition);
    _servoInitialized = true;
    
    Serial.println("\n=== Servo Initialization ===");
    Serial.print("Pin: "); Serial.println(_servoPin);
    Serial.print("Default Position: "); Serial.println(_servoDefaultPosition);
    Serial.print("Min Position: "); Serial.println(_servoMinPosition);
    Serial.print("Max Position: "); Serial.println(_servoMaxPosition);
    Serial.print("Current Speed: "); Serial.println(_servoSpeed);
    Serial.println("===========================\n");
}

void RN_AI::setServoSpeed(int speed) {
    // Store old speed for debug
    int oldSpeed = _servoSpeed;
    
    // Update speed
    _servoSpeed = constrain(speed, 0, 255);
    
    // Calculate delay for debug
    int delayTime = map(_servoSpeed, 0, 255, 50, 5);
    
    Serial.println("\n=== Servo Speed Update ===");
    Serial.print("Old speed: "); Serial.println(oldSpeed);
    Serial.print("New speed: "); Serial.println(_servoSpeed);
    Serial.print("Resulting delay: "); Serial.print(delayTime); Serial.println("ms");
    Serial.println("========================\n");
}

void RN_AI::setServoAngle(int angle) {
    if (!_servoInitialized) {
        _errorState = true;
        _errorCode = ERROR_OUT_OF_RANGE;
        Serial.println("Error: Servo not initialized!");
        return;
    }
    
    angle = constrain(angle, _servoMinPosition, _servoMaxPosition);
    Serial.print("Moving speed: "); Serial.println(_servoSpeed);
    // Calculate delay based on speed (faster speed = shorter delay)
    int delayTime = map(_servoSpeed, 0, 255, 50, 5);  // 50ms at slowest, 5ms at fastest
    Serial.print("Movement delay: "); Serial.println(delayTime);
    
    // Get current position
    int currentPos = _servoMotor.read();
    Serial.print("Starting position: "); Serial.println(currentPos);
    
    // Move servo gradually to target angle
    while (currentPos != angle) {
        if (currentPos < angle) {
            currentPos++;
        } else {
            currentPos--;
        }
        currentPos = constrain(currentPos, _servoMinPosition, _servoMaxPosition);
        _servoMotor.write(currentPos);
        delay(delayTime);
    }
    
    Serial.print("Final position: "); Serial.println(angle);
    Serial.println("=== Servo Movement Complete ===\n");
}

void RN_AI::resetServos() {
    if (_servoInitialized) {
        Serial.println("Resetting servo to default position");
        setServoAngle(_servoDefaultPosition);
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
    // Get current yaw
    float currentYaw = getYaw();
    
    // Calculate angle error and normalize to [-180, 180]
    float angleError = currentYaw - _targetYaw;
    if (angleError > 180) angleError -= 360;
    if (angleError < -180) angleError += 360;

    // Calculate proportional correction
    float correction = _gyroKp * angleError;
    
    // Reverse correction when moving backward
    if (_isBackward) {
        correction = -correction;  // Reverse the correction for backward movement
    }
    
    // Map correction to servo angle range
    int servoAngle = _steeringCenter + map(correction, -180, 180, -_steeringRange, _steeringRange);
    
    // Constrain servo angle to valid range
    servoAngle = constrain(servoAngle, _steeringCenter - _steeringRange, _steeringCenter + _steeringRange);
    
    // Apply steering correction
    _servoMotor.write(servoAngle);
    
    // Debug output
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= 1000) {  // Print every second
        Serial.print("Yaw: "); Serial.print(currentYaw);
        Serial.print("° Error: "); Serial.print(angleError);
        Serial.print("° Correction: "); Serial.print(correction);
        Serial.print(" Servo: "); Serial.print(servoAngle);
        Serial.print(" Direction: "); Serial.println(_isBackward ? "Backward" : "Forward");
        lastPrintTime = currentTime;
    }
}

void RN_AI::moveForwardWithGyro() {
    _isForward = true;
    _isBackward = false;
    _isRotating = false;
    moveForward();
    updateGyro();
    adjustDirectionWithGyro();
    // printGyroMotorStatus();
}

void RN_AI::moveBackwardWithGyro() {
    _isForward = false;
    _isBackward = true;
    _isRotating = false;
    moveBackward();
    updateGyro();
    adjustDirectionWithGyro();
    // printGyroMotorStatus();
}

void RN_AI::stopMovementWithGyro() {
    _isForward = false;
    _isBackward = false;
    _isRotating = false;
    stopMovement();
    printGyroMotorStatus();
}

void RN_AI::printGyroMotorStatus() {
    // Only print if moving with gyro control
    if (!_isForward && !_isBackward) {
        return;
    }

    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    
    // Check if it's time to print (every 1 second)
    if (currentTime - lastPrintTime < 1000) {
        return;
    }
    lastPrintTime = currentTime;

    // Print status information
    Serial.print("Yaw: ");
    Serial.print(getYaw(), 2);
    Serial.print("° Target: ");
    Serial.print(getTargetYaw(), 2);
    Serial.print("° Speed: ");
    Serial.print(_movementSpeed);
    Serial.print(" Direction: ");
    Serial.println(_isForward ? "Forward" : "Backward");
}

void RN_AI::initializeSteeringServo(int pin, int centerPos, int range) {
    _steeringServoPin = pin;
    _steeringCenter = centerPos;
    _steeringRange = range;
    
    // Initialize the servo
    _servoMotor.attach(_steeringServoPin);
    _servoMotor.write(_steeringCenter);
    
    Serial.println("\n=== Steering Servo Initialization ===");
    Serial.print("Pin: "); Serial.println(_steeringServoPin);
    Serial.print("Center Position: "); Serial.println(_steeringCenter);
    Serial.print("Steering Range: "); Serial.println(_steeringRange);
    Serial.println("===================================\n");
}

void RN_AI::updateTargetAngle(float angleDelta) {
    updateGyro();
    // Get current yaw and add the angle delta
    _targetYaw = _yaw + angleDelta;
    
    // Normalize target angle to [0, 360)
    if (_targetYaw >= 360.0) _targetYaw -= 360.0;
    if (_targetYaw < 0.0) _targetYaw += 360.0;
    
    Serial.print("Current Yaw: "); Serial.print(_yaw);
    Serial.print("° Adding: "); Serial.print(angleDelta);
    Serial.print("° New Target: "); Serial.print(_targetYaw);
    Serial.println("°");
}


