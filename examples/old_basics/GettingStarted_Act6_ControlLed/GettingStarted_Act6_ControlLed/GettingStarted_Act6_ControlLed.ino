#include "RN.h"

// Define the pin for the LED
const int LED_PIN = 7;

// Create an instance of the RN class for LED control
RN ledControl(LED_PIN, true);  // true indicates that this is an LED control

void setup() {
  // Initialize serial communication (optional, for debugging)
  Serial.begin(9600);
}

void loop() {
  // Turn on the LED
  ledControl.setLED(true);
  Serial.println("LED is ON");
  delay(1000);  // Wait for 1 second
  
  // Turn off the LED
  ledControl.setLED(false);
  Serial.println("LED is OFF");
  delay(1000);  // Wait for 1 second
}
