#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #if defined(__AVR__)
    #include <avr/io.h>
  #endif
  #include "WProgram.h"
#endif

#include "RN.h"

// Error codes
#define ERROR_NONE 0
#define ERROR_INVALID_SPEED 1
#define ERROR_INVALID_DIRECTION 2
#define ERROR_OUT_OF_RANGE 3
#define ERROR_NO_ECHO 4

// Constructor Implementations
RN::RN(int motorPin1, int motorPin2, int speedPin1) 
    : _motorPin1(motorPin1), _motorPin2(motorPin2), _speedPin1(speedPin1), _motorPin3(-1), _motorPin4(-1), _speedPin2(-1), _errorState(false), _errorCode(ERROR_NONE) {
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    pinMode(_speedPin1, OUTPUT);
}

// Distance sensor constructor
RN::RN(int trigPin, int echoPin) 
    : _trigPin(trigPin), _echoPin(echoPin), _errorState(false), _errorCode(ERROR_NONE) {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

// Color sensor constructor
RN::RN(int s0, int s1, int s2, int s3, int out) 
    : _s0(s0), _s1(s1), _s2(s2), _s3(s3), _sensorOut(out), _errorState(false), _errorCode(ERROR_NONE) {
    pinMode(_s0, OUTPUT);
    pinMode(_s1, OUTPUT);
    pinMode(_s2, OUTPUT);
    pinMode(_s3, OUTPUT);
    pinMode(_sensorOut, INPUT);
}

// Servo constructor
RN::RN(int servoPin) 
    : _servoPin(servoPin), _servoSpeed(0), _servoMinPosition(0), _servoMaxPosition(180), _errorState(false), _errorCode(ERROR_NONE) {
    // Initialization is now done in Servobegin()
}

// LED constructor
RN::RN(int ledPin, bool isLED) 
    : _ledPin(ledPin), _isLEDControl(isLED), _errorState(false), _errorCode(ERROR_NONE) {
    if (_isLEDControl) {
        pinMode(_ledPin, OUTPUT);
        digitalWrite(_ledPin, LOW);  // Ensure LED is off initially
    }
}

// Digital/Analog reading constructor
RN::RN(int pin, bool isDigital, bool isInput)
    : _pin(pin), _isDigital(isDigital), _isInputMode(isInput), _errorState(false), _errorCode(ERROR_NONE) {
    pinMode(_pin, isInput ? INPUT : OUTPUT);
}

// L293D shield constructor
RN::RN(int motorPin1, int motorPin2, int speedPin1, int motorPin3, int motorPin4, int speedPin2) 
    : _motorPin1(motorPin1), _motorPin2(motorPin2), _speedPin1(speedPin1), _motorPin3(motorPin3), _motorPin4(motorPin4), _speedPin2(speedPin2), _errorState(false), _errorCode(ERROR_NONE) {
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    pinMode(_speedPin1, OUTPUT);
    pinMode(_motorPin3, OUTPUT);
    pinMode(_motorPin4, OUTPUT);
    pinMode(_speedPin2, OUTPUT);
    _motorSpeed[0] = 0;
    _motorSpeed[1] = 0;
    _motorSpeed[2] = 0;
    _motorSpeed[3] = 0;
}

// Motor control methods
bool RN::setMotorSpeed(int motorId, int speed) {
    if (motorId < 1 || motorId > 4) {
        _errorState = true;
        _errorCode = ERROR_INVALID_SPEED;
        return false;
    }
    if (speed < 0 || speed > 255) {
        _errorState = true;
        _errorCode = ERROR_INVALID_SPEED;
        return false;
    }
    _motorSpeed[motorId - 1] = speed;
    analogWrite((motorId == 1) ? _motorPin1 : (motorId == 2) ? _motorPin2 : (motorId == 3) ? _motorPin3 : _motorPin4, speed);
    return true;
}

bool RN::setMotorDirection(int motorId, int direction) {
    if (motorId < 1 || motorId > 4) {
        _errorState = true;
        _errorCode = ERROR_INVALID_DIRECTION;
        return false;
    }
    if (direction != FORWARD && direction != BACKWARD) {
        _errorState = true;
        _errorCode = ERROR_INVALID_DIRECTION;
        return false;
    }
    digitalWrite((motorId == 1) ? _motorPin1 : (motorId == 2) ? _motorPin2 : (motorId == 3) ? _motorPin3 : _motorPin4, direction);
    return true;
}

void RN::stopMotor(int motorId) {
    if (motorId < 1 || motorId > 4) return;
    analogWrite((motorId == 1) ? _motorPin1 : (motorId == 2) ? _motorPin2 : (motorId == 3) ? _motorPin3 : _motorPin4, 0);
}

bool RN::increaseMotorSpeed(int motorId, int increment) {
    if (motorId < 1 || motorId > 4 || increment < 0) return false;
    int newSpeed = _motorSpeed[motorId - 1] + increment;
    if (newSpeed > 255) newSpeed = 255;
    setMotorSpeed(motorId, newSpeed);
    return true;
}

bool RN::decreaseMotorSpeed(int motorId, int decrement) {
    if (motorId < 1 || motorId > 4 || decrement < 0) return false;
    int newSpeed = _motorSpeed[motorId - 1] - decrement;
    if (newSpeed < 0) newSpeed = 0;
    setMotorSpeed(motorId, newSpeed);
    return true;
}

void RN::stopAllMotors() {
    stopMotor(1);
    stopMotor(2);
    stopMotor(3);
    stopMotor(4);
}

// Distance sensor methods
float RN::getDistance() {
    // Ensure the trigger is low before sending a pulse
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    // Measure the duration of the echo
    long duration = pulseIn(_echoPin, HIGH);

    // Convert the duration to distance (in cm)
    float distance = duration * 0.0344 / 2;
    return distance;
}

// Color sensor methods
void RN::begin() {
    pinMode(_s0, OUTPUT);
    pinMode(_s1, OUTPUT);
    pinMode(_s2, OUTPUT);
    pinMode(_s3, OUTPUT);
    pinMode(_sensorOut, INPUT);

    digitalWrite(_s0, HIGH);
    digitalWrite(_s1, LOW);
}

unsigned int RN::getIntensityR() {
    digitalWrite(_s2, LOW);
    digitalWrite(_s3, LOW);
    delay(100); // Give some time for the sensor to stabilize
    return pulseIn(_sensorOut, HIGH);
}

unsigned int RN::getIntensityG() {
    digitalWrite(_s2, HIGH);
    digitalWrite(_s3, HIGH);
    delay(100); // Give some time for the sensor to stabilize
    return pulseIn(_sensorOut, HIGH);
}

unsigned int RN::getIntensityB() {
    digitalWrite(_s2, LOW);
    digitalWrite(_s3, HIGH);
    delay(100); // Give some time for the sensor to stabilize
    return pulseIn(_sensorOut, HIGH);
}

// Servo control methods
void RN::Servobegin() {
    Serial.begin(9600);  // Start Serial communication for debugging
    Serial.println("Attaching servo...");
    _servoMotor.attach(_servoPin);
}

bool RN::setServoSpeed(int speed) {
    _servoSpeed = speed;
    return true; // Servo speed management may depend on specific implementation
}

bool RN::setServoPosition(int position) {
    if (position < _servoMinPosition || position > _servoMaxPosition) {
        _errorState = true;
        _errorCode = ERROR_OUT_OF_RANGE;
        Serial.print("Error: Servo position out of range: ");
        Serial.println(position);
        return false;
    }
    Serial.print("Setting servo to position: ");
    Serial.println(position);
    _servoMotor.write(position);
    return true;
}


bool RN::calibrateServo(int minPosition, int maxPosition) {
    _servoMinPosition = minPosition;
    _servoMaxPosition = maxPosition;
    return true;
}

// LED control methods
void RN::setLED(bool state) {
    if (_isLEDControl) {
        digitalWrite(_ledPin, state ? HIGH : LOW);
    }
}

bool RN::controlLED(bool state) {
    setLED(state);
    return true; // Additional control logic may be added
}

// MPU6050 methods
void RN::beginMPU() {
    mpu.initialize();
}

void RN::calcAngle(float& roll, float& pitch) {
    // Example: replace with actual MPU6050 angle calculation
    roll = 0.0;
    pitch = 0.0;
}

float RN::calcYaw() {
    // Example: replace with actual MPU6050 yaw calculation
    return 0.0;
}

float RN::calcPitch() {
    // Example: replace with actual MPU6050 pitch calculation
    return 0.0;
}

// Miscellaneous methods
int RN::getReading() {
    if (_isDigital) {
        return digitalRead(_pin);
    } else {
        return analogRead(_pin);
    }
}

bool RN::isError() {
    return _errorState;
}

int RN::getErrorCode() {
    return _errorCode;
}
