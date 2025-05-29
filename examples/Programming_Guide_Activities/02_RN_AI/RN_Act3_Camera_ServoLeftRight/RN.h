#ifndef RN_H
#define RN_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// Define PWM rate for DC motors
#define DC_MOTOR_PWM_RATE 255 // Adjust as necessary for your motor

// Define settings for different microcontrollers (AVR and PIC32)
// [Insert your microcontroller-specific settings here]

#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

#define SINGLE 1
#define DOUBLE 2
#define INTERLEAVE 3
#define MICROSTEP 4

#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

class AFMotorController
{
public:
    AFMotorController(void);
    void enable(void);
    friend class AF_DCMotor;
    void latch_tx(void);
    uint8_t TimerInitalized;
};

class AF_DCMotor
{
public:
    AF_DCMotor(uint8_t motornum, uint8_t freq = DC_MOTOR_PWM_RATE);
    void run(uint8_t);
    void setSpeed(uint8_t);

private:
    uint8_t motornum, pwmfreq;
};

class AF_Stepper
{
public:
    AF_Stepper(uint16_t, uint8_t);
    void step(uint16_t steps, uint8_t dir, uint8_t style = SINGLE);
    void setSpeed(uint16_t);
    uint8_t onestep(uint8_t dir, uint8_t style);
    void release(void);
    uint16_t revsteps; // # steps per revolution
    uint8_t steppernum;
    uint32_t usperstep, steppingcounter;

private:
    uint8_t currentstep;
};

uint8_t getlatchstate(void);

class RN
{
public:
    // Constructors for different sensor types
    RN(int motorPin1, int motorPin2, int speedPin);
    RN(int trigPin, int echoPin);
    RN(int s0, int s1, int s2, int s3, int out);
    RN(int servoPin); // Constructor for Servo control
    RN(int ledPin, bool isLED); // Constructor for LED control
    RN(int pin, bool isDigital, bool isInput);
    RN(int motorPin1, int motorPin2, int speedPin1, int motorPin3, int motorPin4, int speedPin2); // Constructor for L293D shield

    // Motor control methods
    bool setMotorSpeed(int motorId, int speed); // motorId: 1-4
    bool setMotorDirection(int motorId, int direction); // motorId: 1-4
    void stopMotor(int motorId); // motorId: 1-4
    bool increaseMotorSpeed(int motorId, int increment); // motorId: 1-4
    bool decreaseMotorSpeed(int motorId, int decrement); // motorId: 1-4
    void stopAllMotors(); // Stop all motors

    // Distance sensor methods
    float getDistance();
    float measureDistance(); // Measure distance using the sensor

    // Color sensor methods
    void begin(); // Initialize the color sensor
    unsigned int getIntensityR(); // Get intensity of red color
    unsigned int getIntensityG(); // Get intensity of green color
    unsigned int getIntensityB(); // Get intensity of blue color

    // Servo control methods
    void Servobegin(); // Initialize the servo
    bool setServoSpeed(int speed); // Set speed of the servo
    bool setServoPosition(int position); // Set position of the servo
    bool calibrateServo(int minPosition, int maxPosition); // Calibrate the servo

    // LED control methods
    void setLED(bool state); // Set the LED state
    bool controlLED(bool state); // Control the LED

    // MPU6050 methods
    void beginMPU(); // Initialize the MPU6050 sensor
    void calcAngle(float& roll, float& pitch); // Calculate roll and pitch angles
    float calcYaw(); // Calculate yaw angle
    float calcPitch(); // Calculate pitch angle

    // Miscellaneous methods
    int getReading(); // Get digital or analog reading
    bool isError(); // Check if there is an error
    int getErrorCode(); // Get the error code

private:
    // Motor control pins
    int _motorPin1, _motorPin2, _speedPin1;
    int _motorPin3, _motorPin4, _speedPin2;
    int _motorSpeed[4]; // Array to hold the speed of 4 motors

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

    // LED control
    int _ledPin;
    bool _isLEDControl;

    // Pin configuration
    int _pin;
    bool _isDigital;
    bool _isInputMode;

    // Error state
    bool _errorState;
    int _errorCode; // Store different error codes

    // MPU6050 object
    MPU6050 mpu;
};

#endif
