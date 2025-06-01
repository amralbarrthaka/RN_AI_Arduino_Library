#include <RN.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

HUSKYLENS huskylens; // Create an instance of the HUSKYLENS library
SoftwareSerial mySerial(11, 12); // Create a software serial communication on pins 11 and 12
int ledPin = 13; // Declare the LED pin
int camColorID=0; // Declare a variable to store the color ID

void setup() {
  Serial.begin(115200); // Initialize the Serial communication
  mySerial.begin(9600); // Initialize the software serial communication

  // Try to begin the HUSKYLENS communication
  while (!huskylens.begin(mySerial))
  {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the Protocol Type in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
}

void loop()
{
  Serial.println(getCamColorID()); // Print the color ID over the Serial monitor
}

int getCamColorID()
{
  // Request data from the HUSKYLENS
  if (!huskylens.request())
    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  // Check if the HUSKYLENS has learned any object
  else if(!huskylens.isLearned())
    Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  // Check if there is any block or arrow on the screen
  else if(!huskylens.available())
    Serial.println(F("No block or arrow appears on the screen!"));
  else
  {
    // If there is any block or arrow on the screen, read the data and extract the color ID
    while (huskylens.available())
    {
      HUSKYLENSResult result = huskylens.read(); // Read the data from the HUSKYLENS
      if (result.command == COMMAND_RETURN_BLOCK){
        camColorID=result.ID; // Extract the color ID from the HUSKYLENS data
      }
      else if (result.command == COMMAND_RETURN_ARROW){
        camColorID=result.ID; // Extract the color ID from the HUSKYLENS data
      }
    }
  }
  return(camColorID); // Return the extracted color ID
}