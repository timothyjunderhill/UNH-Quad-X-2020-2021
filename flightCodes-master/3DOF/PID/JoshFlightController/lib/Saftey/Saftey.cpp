
#include <Arduino.h>
//-----------------------------------------------------------------------------------
// THIS SUBROUTINE INCLUDES THE SATFEY FEATURES AND DEBUGGING.
//-----------------------------------------------------------------------------------


bool debug = true; 

// led light 
int led = 13; 

void Blink()
{
	digitalWrite(led,HIGH);
	delay(100);
	digitalWrite(led,LOW);
	delay(100);
}

void SafteyInitialization()
{
    if(debug == true)  
	{
		Serial.begin(115200);
		while(!Serial);
		Serial.println("DEBUGING");
	}

    pinMode(led,OUTPUT);
}

void SetupCompleted()
{
	digitalWrite(led,HIGH);
	Serial.println("Finished Setup");
	delay(1000);
}


extern volatile unsigned int throttle_Pulse;
extern volatile unsigned int activateMotor;

// Make sure all channels are in the right position before turning on
// prevents jump start of the quadcopter 
void controllerCheck()
{
	while(activateMotor > 1100)
	{
		Serial.println("Turn left controller nobe to 1000");
		Blink();
	}

	while(throttle_Pulse > 1100)
	{
		Serial.println("Lower throttle pulse to 1000");
		Blink();
	}
}