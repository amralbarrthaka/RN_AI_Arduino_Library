#ifndef RN_AI_H
#define RN_AI_H

#include <Arduino.h>
#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "HUSKYLENS.h"
#include <Servo.h>

// Define PWM rate for DC motors
#define DC_MOTOR_PWM_RATE 255 // Adjust as necessary for your motor

// Motor direction constants
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

// Servo constants
#define RN_SERVO_DEFAULT_SPEED 0
#define RN_SERVO_DEFAULT_ANGLE 90
#define RN_SERVO_MIN_ANGLE 0
#define RN_SERVO_MAX_ANGLE 180

// Error codes
#define ERROR_NONE 0
#define ERROR_INVALID_SPEED 1
#define ERROR_INVALID_DIRECTION 2
#define ERROR_OUT_OF_RANGE 3
#define ERROR_NO_ECHO 4

class RN_AI
{
private:
    // Motor control pins
    int _motorPin1, _motorPin2, _speedPin1;
    int _motorPin3, _motorPin4, _speedPin2;
    uint8_t _movementSpeed; // Speed for movement functions

    // Distance sensor pins
    int _trigPin, _echoPin;

    // Color sensor pins
    int _s0, _s1, _s2, _s3, _sensorOut;

    // Servo control
    Servo _servoMotor;
    int _servoPin;
    int _servoSpeed;
    int _servoMinPosition;
    int _servoMaxPosition;
    int _servoDefaultPosition;
    bool _servoInitialized;

    // LED control
    int _ledPin;
    bool _isLEDControl;

    // Pin configuration
    int _pin;
    bool _isDigital;
    bool _isInputMode;

    // Error state
    bool _errorState;
    int _errorCode;

    // MPU6050 object
    Adafruit_MPU6050 _mpu;
    
    // Gyro variables
    float _yaw;
    float _correctionValueGyro;
    unsigned long _lastMicros;
    float _lastGyroZ;
    float _targetYaw;
    float _totalCorrectionValueGyro;

    // Gyro movement variables
    bool _isForward;
    bool _isBackward;
    bool _isRotating;
    float _gyroKp;  // Proportional gain for gyro correction
    int _steeringServoPin;  // Pin for steering servo
    int _steeringCenter;    // Center position of steering servo
    int _steeringRange;     // Range of steering movement

public:
    // Constructors
    RN_AI();
    RN_AI(int motorPin1, int motorPin2, int speedPin);
    RN_AI(int trigPin, int echoPin);
    RN_AI(int s0, int s1, int s2, int s3, int out);
    RN_AI(int servoPin);
    RN_AI(int ledPin, bool isLED);
    RN_AI(int pin, bool isDigital, bool isInput);
    RN_AI(int motorPin1, int motorPin2, int speedPin1, int motorPin3, int motorPin4, int speedPin2);

    // Motor control methods
    bool begin();
    void initializeMotor(int pin1, int pin2, int speedPin);
    void setMovementSpeed(float speed);
    void moveForward();
    void moveBackward();
    void stopMovement();
    bool setMotorDirection(int motorId, int direction);
    void stopMotor(int motorId);
    bool increaseMotorSpeed(int motorId, int increment);
    bool decreaseMotorSpeed(int motorId, int decrement);
    void stopAllMotors();

    // Servo control methods
    void initializeServo(int pin, int defaultPos, int minPos, int maxPos);
    void setServoSpeed(int speed);
    void setServoAngle(int angle);
    bool calibrateServo(int minPosition, int maxPosition);
    void resetServos();

    // Distance sensor methods
    float getDistance();
    float measureDistance();

    // Color sensor methods
    void beginColorSensor();
    unsigned int getIntensityR();
    unsigned int getIntensityG();
    unsigned int getIntensityB();

    // LED control methods
    void setLED(bool state);
    bool controlLED(bool state);

    // MPU6050 methods
    void initializeGyro();
    void correctGyro();
    void updateGyro();
    void setGyroRange(mpu6050_gyro_range_t range);
    void setFilterBandwidth(mpu6050_bandwidth_t bandwidth);
    float getYaw() { return _yaw; }
    float getTargetYaw() { return _targetYaw; }
    void setTargetYaw(float yaw) { _targetYaw = yaw; }

    // Miscellaneous methods
    int getReading();
    bool isError();
    int getErrorCode();

    // Gyro movement methods
    void setGyroKp(float kp) { _gyroKp = kp; }
    float getGyroKp() { return _gyroKp; }
    void initializeSteeringServo(int pin, int centerPos, int range);
    void adjustDirectionWithGyro();
    void moveForwardWithGyro();
    void moveBackwardWithGyro();
    void stopMovementWithGyro();
    void printGyroMotorStatus();
    void updateTargetAngle(float angleDelta);
};

#endif
