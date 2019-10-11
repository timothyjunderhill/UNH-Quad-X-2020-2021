// ESC_CALIBRATION 
// 1. UPLOAD THE CODE TO TEENSY 3.5 
// 2. UNPLUG CORD TO TEENSY 
// 3. TURN RECIEVER ON AND MOVE TO FULL THROTTLE 
// 4. WAIT FOR MOTORS TO FINISH BEEPING 
// 5. MOVE THORTTLE TO LOW POSITION
// 6. ESC ARE NOW CALIBRATED 
// 7. MOVE THROTTLE UP AGAIN TO TEST CALIBRATION
// 8. IF MOTORS DO NOT ALL START AT THE SAME TIME START OVER
// Created by: Joshua Wallace 
// Date: 7/18/19

// Libaries 
#include <Arduino.h>
#include <Wire.h>

//Receiver Pin
const int ch3 = 26; 		//Left Stick - Vertical (Throttle)

//Motor Pins(or esc)
const int motorPin1 = 6;
const int motorPin2 = 10;
const int motorPin3 = 5;
const int motorPin4 = 20;
int motorPWM = 0;

// Interrupt  Variables 
elapsedMicros time;  
volatile int R = 0;
unsigned long timer = 0;
unsigned long lastUpdate = 0;
unsigned long updateTime = 4000;

// Extra 
const int freq = 250;
const int led = 13;
const int maxPulse = 4000;
const int maxPWM = 256;


//External Interrupt to record pulse length 
void ch3Int(){ 	if (digitalReadFast(ch3)) timer = time;  else R = time - timer;}


void setup() {

	// Set Pin direction 
	pinMode(motorPin1,OUTPUT);
	pinMode(motorPin2,OUTPUT);
	pinMode(motorPin3,OUTPUT);
	pinMode(motorPin4,OUTPUT);
	pinMode(led,OUTPUT);

	// Initialize interrupt 
	attachInterrupt(ch3,ch3Int,CHANGE);

	// Initialize the PWM signal 
	analogWriteFrequency(motorPin1, freq); 
	analogWriteResolution(8); // set the value from 0 - 256

	// Hold until throttle is in maximum position to prevent improper intilization 
	while(R<1950)
	{
		digitalWrite(led,HIGH);
		delay(250);
		digitalWrite(led,LOW);
		delay(250);
	}

	// Finished Setup 
	digitalWrite(led,HIGH);
	delay(250);
}

void loop() 
{

	// Wait for 4000 us to update motors again 
	if((time - lastUpdate) > updateTime)
	{
		// Convert pulse to PWM value 
		motorPWM = R * maxPWM/maxPulse;

		// Spin the motors 
		analogWrite(motorPin1,motorPWM);
		analogWrite(motorPin2,motorPWM);
		analogWrite(motorPin3,motorPWM);
		analogWrite(motorPin4,motorPWM);
	}
}
