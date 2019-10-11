// LIBRARIES 
#include <Arduino.h>
#include "ActualAttitude.hpp"
#include "DesiredAttitude.hpp"
#include "AttitudeController.hpp"
#include "Safety.hpp"
#include "SpinMotors.hpp"
#include "ModeDetection.hpp"

// attitude loop
const int updateTime = 4000;
long lastUpdate = 0;

// Printing
long lastPrint = 0;
const int printTimer = 5000;

//-----------------------
// DEBUGGING
//-----------------------
// Wait for Serial 
bool debug = false;

// Print Angles, Print Rates 
bool angle = false;
bool actualRates = false;
bool accelerations = false;
bool reciever = false; 
bool desiredRates = false;
bool pulseOutput = true;
bool state = false; 

elapsedMicros elapsedTime;

// GLOBAL VARIABLES USED FOR DEBUGGING
extern float actualPitch, actualRoll, actualYaw;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern int desiredPitchRate, desiredRollRate, desiredYawRate;
extern int escPulse1, escPulse2, escPulse3, escPulse4;
extern float offsetPitchRate, offsetRollRate, offsetYawRate; 
extern int pitchPulse, rollPulse, yawPulse;
extern float accelerationX, accelerationY, accelerationZ, averageZ, magAcceleration;
extern bool startMotor;
extern int flightMode;

//----------------------
// INTERURUPTS  
//-----------------------
unsigned long timer[7] = {0}; 	          //Timer [us]
volatile int R[7] = {0};	              //Hand-Held Receiver Signals [us]

// Reciever  
const int ch1 = 24;
const int ch2 = 25;
const int ch3 = 26;
const int ch4 = 27;
const int ch5 = 28;
const int ch6 = 29;

 // Roll 
 void ch1Int()
{
  if (digitalReadFast(ch1))
	timer[1] = elapsedTime;
  else
	R[1] = elapsedTime - timer[1];
}

// Pitch 
void ch2Int()
{
  if (digitalReadFast(ch2))
  	timer[2] = elapsedTime;
  else
	R[2] = elapsedTime - timer[2];
}

// Throttle 
void ch3Int()
{
   if (digitalReadFast(ch3))
   	timer[3] = elapsedTime;
   else
	R[3] = elapsedTime - timer[3];
}

// Yaw 
void ch4Int()
{
   if (digitalReadFast(ch4))
   	timer[4] = elapsedTime;
   else
   	R[4] = elapsedTime - timer[4];
}

// Switch 
void ch5Int()
{
   if (digitalReadFast(ch5))
	timer[5] = elapsedTime;
   else
	R[5] = elapsedTime - timer[5];
}

void ch6Int()
{
   if (digitalReadFast(ch6))
	timer[6] = elapsedTime;
   else
	R[6] = elapsedTime - timer[6];
}

void RecieverIntilization()
{
	// Set reciever pin direction 
	pinMode(ch1, INPUT);
	pinMode(ch2, INPUT);
	pinMode(ch3, INPUT);
	pinMode(ch4, INPUT);
	pinMode(ch5, INPUT);
	pinMode(ch6, INPUT);

	attachInterrupt(ch1, ch1Int, CHANGE);
	attachInterrupt(ch2, ch2Int, CHANGE);
	attachInterrupt(ch3, ch3Int, CHANGE);
	attachInterrupt(ch4, ch4Int, CHANGE);
	attachInterrupt(ch5, ch5Int, CHANGE);
	attachInterrupt(ch6, ch6Int, CHANGE);

	Serial.println("Interrupts Initialized");
}

//-----------------------------------
//IMU CALIBRATION 
//-----------------------------------
float offsetPitchRate = 0, offsetRollRate = 0, offsetYawRate = 0;
float sumPitchRate = 0, sumRollRate = 0, sumYawRate = 0;
int i = 0;
float offsetAccX, offsetAccY, offsetAccZ;
float sumAccX = 0, sumAccY = 0, sumAccZ = 0;

void IMUCalibration()
{
	while( i < 2000)
	{
		if( (elapsedTime - lastUpdate) > updateTime)
		{
			lastUpdate = elapsedTime;
			i++;
			GetActualAttitude();
			sumPitchRate += actualPitchRate;
			sumRollRate += actualRollRate;
			sumYawRate += actualYawRate; 
			sumAccX += accelerationX;
			sumAccY += accelerationY;
			sumAccZ += accelerationZ;
		}
	}
	offsetPitchRate = sumPitchRate/2000;
	offsetRollRate = sumRollRate/2000;
	offsetYawRate = sumYawRate/2000;
	offsetAccX = sumAccX/2000;
	offsetAccY = sumAccY/2000;
	offsetAccZ = sumAccZ/2000 - 9.81;
	Serial.println("Finished Calibration");
}


////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() 
{ 
 	if (debug == true)
	{
		Serial.begin(115200);

		// Wait for serial monitor to activate
		while(!Serial);
	}
	

	RecieverIntilization();
	
	IMUInitialization();

	IMUCalibration();

	EscInitialize();

	SafteyInitialization();
}

////////////////////////////////////////////////
// Main
///////////////////////////////////////////////
void loop() 
{

	if ((elapsedTime - lastUpdate) > updateTime)
	{
	 	lastUpdate = elapsedTime;
		GetMode();
	 	GetActualAttitude();
		GetDesiredAttitude();
		GetAttitudeController();
		BoundPulse();
		PulsetoPWM();
	}
	if( debug == true )
	{
		if ((elapsedTime - lastPrint) >= printTimer)
		{ 
			lastPrint = elapsedTime;
			// Test IMU 
			if (angle == true)
			{
				Serial.print("Time Elapsed ");
	 			Serial.print(elapsedTime);
				Serial.print(" Pitch: ");
				Serial.print(actualPitch);
				Serial.print(" Roll: ");
				Serial.print(actualRoll);
				Serial.print(" Yaw: ");
				Serial.print(actualYaw);
				Serial.println();
			}
			// Test IMU
			if (actualRates == true)
			{
				Serial.print("Time Elapsed ");
	 			Serial.print(elapsedTime);
				Serial.print(" PitchRate: ");
  				Serial.print(actualPitchRate);
  				Serial.print(" RollRate: ");
  				Serial.print(actualRollRate);
  				Serial.print(" YawRate: ");
  				Serial.print(actualYawRate);
				Serial.println();
			}
			if (accelerations == true)
			{
				// Serial.print(" Acc X: ");
				// Serial.println(accelerationX);
				// Serial.print(" Acc Y: ");
				// Serial.println(accelerationY);
				// Serial.print(" Acc Z: ");
				// Serial.println(accelerationZ);
				// Serial.print(" Acc Zave: ");
				Serial.print(averageZ);
				Serial.println();
			}
			// Test Reciever 
			if(reciever == true)
			{
				Serial.print("Time Elapsed: ");
	 			Serial.print(elapsedTime);
				Serial.print(" Channel 1 ");
				Serial.print(R[1]);
				Serial.print(" Channel 2 ");
				Serial.print(R[2]);
				Serial.print(" Channel 3 ");
				Serial.print(R[3]);
				Serial.print(" Channel 4 ");
				Serial.print(R[4]);
				Serial.print(" Channel 5 ");
				Serial.print(R[5]);
				Serial.print(" Channel 6 ");
				Serial.print(R[6]);
				Serial.println();
			}
			if(desiredRates == true)
			{
				Serial.print("Time Elapsed ");
	 			Serial.print(elapsedTime);
				Serial.print(" PitchRate: ");
  				Serial.print(desiredPitchRate);
  				Serial.print(" RollRate: ");
  				Serial.print(desiredRollRate);
  				Serial.print(" YawRate: ");
  				Serial.print(desiredYawRate);
				Serial.println();
			}
			if(pulseOutput == true)
			{
				Serial.print("Time Elapsed ");
	 			Serial.print(elapsedTime);
				Serial.print(" Motor1: ");
  				Serial.print(escPulse1);
  				Serial.print(" Motor2: ");
  				Serial.print(escPulse2);
  				Serial.print(" Motor3: ");
  				Serial.print(escPulse3);
				Serial.print(" Motor4: ");
  				Serial.print(escPulse4);
				Serial.println();
			}
			if(state == true)
			{
				Serial.print("Time Elapsed ");
	 			Serial.print(elapsedTime);
				Serial.print(" flightmode ");
  				Serial.print(flightMode);
  				Serial.print(" startMotor: ");
  				Serial.print(startMotor);
				Serial.println();
			}
		}
	}
}

