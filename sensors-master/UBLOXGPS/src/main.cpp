#include <Arduino.h>
#include "UBLOX.h"

/* Make sure the GPS has been set up with ublox software center. 
Set ublox to output UBX-NAV-PVT packets. Additional options for 
gps frequency and buad rate.*/

// Delare uBlox object
UBLOX gps(Serial1,115200);

// Variables 
int fix;

void setup() {

  // Begin connection with serial monitor 
  Serial.begin(115200);

  // Wait for Serial status to turn true 
  while(!Serial);

  // Start the Serial connection for the GPS
  gps.begin();
}

void loop() {

  // Check to see if parsed data is ready 
  if(gps.readSensor()){
    Serial.print("Fix: ");
    Serial.print(gps.getFixType()); // get fix 
    Serial.print(" Latitude: ");
    Serial.print(gps.getLatitude_deg(),10); // print latitude
    Serial.print(" Longitude: ");
    Serial.println(gps.getLongitude_deg(),10); // print longitude 
  }





}