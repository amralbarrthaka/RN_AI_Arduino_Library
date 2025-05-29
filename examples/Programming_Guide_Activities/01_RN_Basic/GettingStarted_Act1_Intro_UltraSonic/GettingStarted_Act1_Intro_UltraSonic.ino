#include "RN.h"

RN sensor(10, 11); // create an instance of RN class with trig pin 14 and echo pin 15

void setup() {
  Serial.begin(9600);
}

void loop() {
  //Get the distance using the sensor and assign it to a variable 
  float distance = sensor.getDistance();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(1000);
}
