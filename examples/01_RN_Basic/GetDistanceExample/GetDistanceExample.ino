#include <RN_Sport.h>

// Create robot instance
RN_Sport robot;

// Distance sensor pins
const int TRIG_PIN = 24;  // Trigger pin
const int ECHO_PIN = 25;  // Echo pin

void setup() {
    Serial.begin(115200);
    Serial.println("Distance Measurement Example");
    
    // Initialize distance sensor
    robot.initializeDistanceSensor(TRIG_PIN, ECHO_PIN);
}

void loop() {
    // Get distance measurement
    float distance = robot.getDistance();
    
    // Print distance
    Serial.print("Distance: ");
    Serial.println(distance);
    
    // Wait 100ms between measurements
    delay(100);
}
