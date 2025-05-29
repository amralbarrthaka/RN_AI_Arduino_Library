
#include <RN.h>

// Create an instance of the RN class with MPU6050 sensor
RN myRobot(1, 2, 3, 4, 5, 6); // Adjust pin numbers according to your setup

void setup() {
    Serial.begin(9600);

    // Initialize MPU6050
    myRobot.beginMPU();

    // Check for initialization errors
    if (myRobot.isError()) {
        Serial.print("Error initializing MPU6050: ");
        Serial.println(myRobot.getErrorCode());
        return;
    }
}

void loop() {
    // Variables to hold the roll and pitch angles
    float roll, pitch;

    // Calculate roll and pitch angles
    myRobot.calcAngle(roll, pitch);

    // Print angles to Serial Monitor
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" degrees, Pitch: ");
    Serial.println(pitch);
    Serial.println(" degrees");

    // Delay to make the output readable
    delay(500);
}
