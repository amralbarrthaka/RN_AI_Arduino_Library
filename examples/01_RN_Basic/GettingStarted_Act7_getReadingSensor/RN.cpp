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
        _errorCode = ERROR_OUT_OF_RANGE;
        return false;
    }
    if (speed < 0 || speed > 255) {
        _errorState = true;
        _errorCode = ERROR_INVALID_SPEED;
        return false;
    }

    _motorSpeed[motorId - 1] = speed;
    int pwmPin = (motorId == 1) ? _speedPin1 : (motorId == 2) ? _speedPin1 : (motorId == 3) ? _speedPin2 : _speedPin2;
    analogWrite(pwmPin, speed);
    return true;
}

bool RN::setMotorDirection(int motorId, int direction) {
    int motorPinA = (motorId == 1) ? _motorPin1 : (motorId == 2) ? _motorPin1 : (motorId == 3) ? _motorPin3 : _motorPin3;
    int motorPinB = (motorId == 1) ? _motorPin2 : (motorId == 2) ? _motorPin2 : (motorId == 3) ? _motorPin4 : _motorPin4;

    switch (direction) {
        case FORWARD:
            digitalWrite(motorPinA, HIGH);
            digitalWrite(motorPinB, LOW);
            break;
        case BACKWARD:
            digitalWrite(motorPinA, LOW);
            digitalWrite(motorPinB, HIGH);
            break;
        case BRAKE:
            digitalWrite(motorPinA, HIGH);
            digitalWrite(motorPinB, HIGH);
            break;
        case RELEASE:
            digitalWrite(motorPinA, LOW);
            digitalWrite(motorPinB, LOW);
            break;
        default:
            _errorState = true;
            _errorCode = ERROR_INVALID_DIRECTION;
            return false;
    }
    return true;
}

void RN::stopMotor(int motorId) {
    if (motorId < 1 || motorId > 2) return;

    switch (motorId) {
        case 1:
            digitalWrite(_motorPin1, LOW);
            digitalWrite(_motorPin2, LOW);
            analogWrite(_speedPin1, 0);
            break;
        case 2:
            digitalWrite(_motorPin3, LOW);
            digitalWrite(_motorPin4, LOW);
            analogWrite(_speedPin2, 0);
            break;
    }
}


bool RN::increaseMotorSpeed(int motorId, int increment) {
    if (motorId < 1 || motorId > 4) {
        _errorState = true;
        _errorCode = ERROR_OUT_OF_RANGE;
        return false;
    }
    int newSpeed = _motorSpeed[motorId - 1] + increment;
    return setMotorSpeed(motorId, min(255, newSpeed));
}

bool RN::decreaseMotorSpeed(int motorId, int decrement) {
    if (motorId < 1 || motorId > 4) {
        _errorState = true;
        _errorCode = ERROR_OUT_OF_RANGE;
        return false;
    }
    int newSpeed = _motorSpeed[motorId - 1] - decrement;
    return setMotorSpeed(motorId, max(0, newSpeed));
}

void RN::stopAllMotors() {
    for (int i = 1; i <= 4; i++) {
        stopMotor(i);
    }
}

// Distance sensor methods
float RN::getDistance() {
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

float RN::measureDistance() {
    return getDistance();
}

// Color sensor methods
void RN::begin() {
    digitalWrite(_s0, HIGH);
    digitalWrite(_s1, LOW);
}

unsigned int RN::getIntensityR() {
    digitalWrite(_s2, LOW);
    digitalWrite(_s3, LOW);
    return pulseIn(_sensorOut, LOW);
}

unsigned int RN::getIntensityG() {
    digitalWrite(_s2, HIGH);
    digitalWrite(_s3, HIGH);
    return pulseIn(_sensorOut, LOW);
}

unsigned int RN::getIntensityB() {
    digitalWrite(_s2, LOW);
    digitalWrite(_s3, HIGH);
    return pulseIn(_sensorOut, LOW);
}

// Servo control methods
void RN::Servobegin() {
    _servoMotor.attach(_servoPin);
}

bool RN::setServoSpeed(int speed) {
    if (speed < 0 || speed > 180) {
        _errorState = true;
        _errorCode = ERROR_INVALID_SPEED;
        return false;
    }
    _servoSpeed = speed;
    _servoMotor.write(_servoSpeed);
    return true;
}

bool RN::setServoPosition(int position) {
    if (position < _servoMinPosition || position > _servoMaxPosition) {
        _errorState = true;
        _errorCode = ERROR_OUT_OF_RANGE;
        return false;
    }
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
    if (_isLEDControl) {
        digitalWrite(_ledPin, state ? HIGH : LOW);
        return true;
    }
    return false;
}

// MPU6050 methods
void RN::beginMPU() {
    Wire.begin();
    mpu.initialize();
}

void RN::calcAngle(float& roll, float& pitch) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    roll = atan2(ay, az) * 180.0 / M_PI;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
}

float RN::calcYaw() {
    // Yaw calculation logic
    return 0.0;
}

float RN::calcPitch() {
    // Pitch calculation logic
    return 0.0;
}

// Miscellaneous methods
int RN::getReading() {
    if (_isDigital) {
        return digitalRead(_pin);
    }
    return analogRead(_pin);
}

bool RN::isError() {
    return _errorState;
}

int RN::getErrorCode() {
    return _errorCode;
}
