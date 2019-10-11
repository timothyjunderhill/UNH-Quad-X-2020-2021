#include <Arduino.h>
#include "imu.hpp"

/////////////////////////////////////////////////////////
//GLOBAL VARIABLES 
/////////////////////////////////////////////////////////
elapsedMicros time;

// Variables for debugging
const int printTimer = 10000;
int lastPrint = 0;

// Filter sample rate
const int updateTime = 4000;
int lastUpdate = 0;

// IMU variable outputs 
extern float pitch, roll, yaw;
extern float pitchD, rollD, yawD;

void setup() {

  // Connect to serial monitor 
  Serial.begin(115200);
  while(!Serial);
  Serial.println("DEBUGING");

  // Intilizise the IMU 
  ImuIntilization();

  // Setup completed 
  Serial.println("Finished Setup");
  delay(1000);

}

void loop() 
{
	// Update pulse to motors every 250hz
	if ((time - lastUpdate) > updateTime)
	{
		lastUpdate = time;

		// Get the rates and angles
		GetIMU();
	}

	if ((time - lastPrint) >= printTimer)
	{
		lastPrint = time;
		Serial.print(rollD);
		Serial.print(", ");
		Serial.print(pitchD);
		Serial.print(", ");
		Serial.print(yawD);
		Serial.print(", ");
		Serial.print(roll);
		Serial.print(", ");
		Serial.print(pitch);
		Serial.print(", ");
		Serial.println(yaw);

	}
}