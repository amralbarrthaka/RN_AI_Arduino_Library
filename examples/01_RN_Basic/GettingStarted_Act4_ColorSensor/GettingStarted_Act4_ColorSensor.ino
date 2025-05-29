#include "RN.h"

// Define the pins connected to the TCS3200 color sensor
const int S0 = 3;
const int S1 = 4;
const int S2 = 5;
const int S3 = 6;
const int sensorOut = 7;

// Create an instance of the RN library
RN colorSensor(S0, S1, S2, S3, sensorOut);

void setup() {
  Serial.begin(9600);
  colorSensor.begin();
}

void loop() {
  // Using the color sensor function to get the intensity of each color
  unsigned int redValue = colorSensor.getIntensityR();
  unsigned int greenValue = colorSensor.getIntensityG();
  unsigned int blueValue = colorSensor.getIntensityB();
  
  Serial.print("Red: ");
  Serial.print(redValue);
  Serial.print(" Green: ");
  Serial.print(greenValue);
  Serial.print(" Blue: ");
  Serial.println(blueValue);
  
  delay(1000); // Delay before the next reading
}
