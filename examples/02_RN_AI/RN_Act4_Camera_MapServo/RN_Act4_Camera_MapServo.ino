#include "HUSKYLENS.h"
#include "RN.h"

HUSKYLENS huskylens;
int ledPin = 13;
void printResult(HUSKYLENSResult result);

// Define the pin connected to the servo motor
const int servoPin = 2;
const int servoMinAngle = 0;   // minimum servo angle
const int servoMaxAngle = 90;  // maximum servo angle
int servoPosition = (servoMinAngle + servoMaxAngle) / 2; // starting position of the servo (center)

// Create an instance of the RN library
RN servoController(servoPin);

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // استخدام Serial1 بدلاً من SoftwareSerial
  pinMode(ledPin, OUTPUT); 
  servoController.Servobegin();
  servoController.setServoPosition(servoPosition); // Set initial position to center
}

void loop() {
  while (!huskylens.begin(Serial1)) { // استخدام Serial1 للتواصل مع HUSKYLENS
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the Protocol Type in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }

  if (!huskylens.request()) {
    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  } else if (!huskylens.isLearned()) {
    Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  } else if (!huskylens.available()) {
    Serial.println(F("No block or arrow appears on the screen!"));
  } else {
    Serial.println(F("###########"));
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      handleResult(result);
      printResult(result);
    }
  }
}

void handleResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    int xCenter = result.xCenter;
    // Convert xCenter (0 to 320) to servo angle (0 to 90)
    servoPosition = map(xCenter, 0, 320, servoMinAngle, servoMaxAngle);
    servoController.setServoPosition(servoPosition);
  }
}

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.ID);
  } else {
    Serial.println("Object unknown!");
  }
}
