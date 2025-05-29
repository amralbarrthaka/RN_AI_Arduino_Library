#include "RN.h"

// Create an RN object for digital or analog reading
// Use isDigital = true for digital, isDigital = false for analog
RN sensor(A0, false, true);  // Digital input on pin A0 or Else
//RN sensor(A0, false, true);  // Analog input on pin A0

void setup() {
    Serial.begin(9600);  // Start serial communication at 9600 baud
}

void loop() {
    // Get the reading from the sensor
    int reading = sensor.getReading();

    // Print the reading to the Serial Monitor
    Serial.print("Sensor Reading: ");
    Serial.println(reading);

    // Add a delay for
}
