/*
 * MPU6050 XY Angle Measurement Example using RN_AI Library
 * 
 * This example demonstrates how to use the RN_AI library to measure
 * angles using the MPU6050 sensor.
 * 
 * Hardware Connections:
 * - MPU6050 VCC to 5V or 3.3V
 * - MPU6050 GND to GND
 * - MPU6050 SCL to Arduino SCL
 * - MPU6050 SDA to Arduino SDA
 */

#include <RN_AI.h>

// Create RN_AI object with default settings
RN_AI robot;

// Variables for timing
unsigned long lastMicros = 0;

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("MPU6050 XY Angle Measurement Example");
  
  // Initialize the robot and MPU6050
  if (!robot.begin()) {
    Serial.println("Failed to initialize RN_Sport!");
    while (1) {
      delay(10);
    }
  }
  robot.initializeGyro();
}

void loop() {
  // Update the yaw angle
  robot.updateGyro();
  
  // Get the current yaw angle
  float yaw = robot.getYaw();
  
  // Print the angle
  Serial.print("Yaw (Heading Estimate): ");
  Serial.print(yaw, 2);
  Serial.println("°");


}
