/* ////////////////////////////////////// 
 * //FLIGHT CONTROLLER
 * //////////////////////////////////////
 * 
 * ////////////////////////////////////// 
 * // HARDWARE
 * ////////////////////////////////////// 
 * 
 * MICROCONTROLLER: TEENSY 3.5 
 * 
 * HAND HELD CONTROLLER: https://www.amazon.com/FlySky-FS-i6-M2-2-4GHz-6-Channel-Transmitter/dp/B00PF160IK
 * 
 * MAKE SURE TO SET THE FAIL SAFE MODE ON THE CONTROLLER AND CHANNEL FIVE IS SWITCH TO THE SWITCH
 * REFER TO THIS VIDEO: https://www.youtube.com/watch?v=LXTEXqR_ghI
 * 
 * IMU: NXP 9DOF ADIAFRUIT 
 * https://www.adafruit.com/product/3463
 * BE SURE TO CALIBRATE AND TEST BEFORE USE https://www.youtube.com/watch?v=1geo_N5X7ic
 * 
 * MOTOR: Turnigy Multistar 2213-980Kv 14 Pole Multi-Rotor Outrunner V2
 * https://hobbyking.com/en_us/turnigy-multistar-2213-980kv-14-pole-multi-rotor-outrunner-v2.html
 * 
 * PROPELLERS: 10x4.5 Inch 
 * https://www.amazon.com/uxcell-Propellers-Fixed-Wing-Airplane-Adapter/dp/B07PXKKV3G/ref=sr_1_6?crid=3S7H28DITMXYT&keywords=1045+propeller&qid=1555036892&s=gateway&sprefix=1045+p%2Caps%2C129&sr=8-6
 * 
 * FRAME: 4-Axis Multi Rotor Airframe 450mm
 * https://www.amazon.com/ShareGoo-Airframe-FrameWheel-Quadcopter-Aircraft/dp/B07H3WDSX3/ref=sr_1_fkmrnull_1_sspa?keywords=4-Axis+Multi+Rotor+Airframe+450mm&qid=1555038824&s=gateway&sr=8-1-fkmrnull-spons&psc=1
 * 
 *  
*/

/////////////////////////////////////////////////////////
//LIBARIES 
/////////////////////////////////////////////////////////
#include <Arduino.h> 							//Used for Visual Studio's code 
#include <ActualAttitude.hpp>     
#include <DesiredAttitude.hpp>
#include <AttitudeController.hpp>
#include <SpinMotors.hpp>
#include <Saftey.cpp> 


elapsedMicros elapsedTime;
extern bool debug;

// attitude loop timing 
long lastUpdate = 0;
int updateTime = 4000;

// Print loop timing
long lastPrint = 0;
int printTimer = 5000;

// Important variables from other subroutines 
extern int desiredPitchRate;
extern float actualPitch, actualRoll, actualYaw;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern float Ipitch, Iroll, Iyaw;
extern float offsetPitch_rate, offsetRoll_rate, offsetYaw_rate;


 /////////////////////////////////////////////////////////
 // INTERUPTS
 /////////////////////////////////////////////////////////
 // The interupts are used to obtain the pulse lengths from the hand held controller

 // Timing Variables for Pulse Width
 unsigned long prev1 = 0;
 volatile unsigned int roll_ratePulse = 1500;
 unsigned long prev2 = 0;
 volatile unsigned int pitch_ratePulse = 1500;
 unsigned long prev3 = 0;
 volatile unsigned int throttle_Pulse = 1500;
 unsigned long prev4 = 0;
 volatile unsigned int yaw_ratePulse = 1500;
 unsigned long prev5 = 0;
 volatile unsigned int activateMotor = 1500;
 unsigned long prev6 = 0;
 volatile unsigned int SWB = 1500;


 // Reciever pins 
 int ch1 = 24;
 int ch2 = 25;
 int ch3 = 26;
 int ch4 = 27;
 int ch5 = 28;
 int ch6 = 29;

 // Get pulse length 

 // Roll 
 void ch1Int()
{
  if (digitalReadFast(ch1))
  	prev1 = elapsedTime;
  else
  	roll_ratePulse = elapsedTime - prev1;

  return;
}

// Pitch 
void ch2Int()
{
  if (digitalReadFast(ch2))
  	prev2 = elapsedTime;
  else
  	pitch_ratePulse = elapsedTime - prev2;

 return; 
}

// Throttle 
void ch3Int()
{
   if (digitalReadFast(ch3))
   	prev3 = elapsedTime;
   else
   	throttle_Pulse = elapsedTime - prev3;

   return;
}

// Yaw 
void ch4Int()
{
   if (digitalReadFast(ch4))
   	prev4 = elapsedTime;
   else
   	yaw_ratePulse = elapsedTime - prev4;
}

// Switch 
void ch5Int()
{
   if (digitalReadFast(ch5))
   	prev5 = elapsedTime;
   else
   	activateMotor = elapsedTime - prev5;
	
   return;
}

// Switch 
void ch6Int()
{
   if (digitalReadFast(ch6))
   	prev6 = elapsedTime;
   else
   	SWB = elapsedTime - prev6;
	
   return;
}

void InteruptInitialization()
 {
	// Reciever 
	pinMode(ch1,INPUT);
	pinMode(ch2,INPUT);
	pinMode(ch3,INPUT);
	pinMode(ch4,INPUT);
	pinMode(ch5,INPUT);

	 // Setup rx pin interrupts
  	attachInterrupt(ch1,ch1Int,CHANGE);
  	attachInterrupt(ch2,ch2Int,CHANGE);
  	attachInterrupt(ch3,ch3Int,CHANGE);
  	attachInterrupt(ch4,ch4Int,CHANGE);
	attachInterrupt(ch5,ch5Int,CHANGE);
	attachInterrupt(ch6,ch6Int,CHANGE);

	return;
 }


/////////////////////////////////////////////////////////
// SETUP
/////////////////////////////////////////////////////////
// Get ready to fly

void setup() 
{
	SafteyInitialization();

	InteruptInitialization();

	controllerCheck();

	imuIntilization();

	escInitialize();

	SetupCompleted();
}

/////////////////////////////////////////////////////////
// MAIN CODE LOOP 
/////////////////////////////////////////////////////////

void loop() 
{
	// Update pulse to motors every 250hz
	if ((elapsedTime - lastUpdate) > updateTime)
	{

		lastUpdate = elapsedTime;

		// Grabs the angles and rates from the IMU
		GetActualAttitude();

		// Grabs the desired rates from the hand held device 
		GetDesiredAttitude();

		// Get the PID corrections and calculate the motor pulses 
		GetAttitudeController();

		// Make sure pulse is in correct range and kill or activate motors  
		BoundPulse();

		// Convert the Pulse to PWM in order to spin the motors 
		pulsetoPWM();
	}	

	// Need to find a way to put this into saftey subroutine 
	 if (debug == true)
    {
        if ((elapsedTime - lastPrint) >= printTimer)
        {
            lastPrint = elapsedTime;
			Serial.print(actualPitch);
			Serial.print(", ");
			Serial.print(actualRoll);
			Serial.print(", ");
			Serial.println(actualYaw);

        }
	}
}