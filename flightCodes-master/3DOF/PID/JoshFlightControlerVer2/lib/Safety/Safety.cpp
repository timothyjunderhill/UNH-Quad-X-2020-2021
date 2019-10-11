//---------------------------------------------------------------
//SAFETY FEATURES ARE PROVIDED BELOW TO DECREASE CHANCE OF INJURY 
//---------------------------------------------------------------
#include <Arduino.h>

const int led = 13; 
// GLOBAL VARIABLES
extern float actualPitch, actualRoll, actualYaw;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern int desiredPitchRate, desiredRollRate, desiredYawRate;
extern int escPulse1, escPulse2, escPulse3, escPulse4;
extern float offsetPitchRate, offsetRollRate, offsetYawRate; 
extern volatile int R[7];
extern int pitchPulse, rollPulse, yawPulse;
extern float accelerationX, accelerationY, accelerationZ, averageZ, magAcceleration;
extern float offsetAccX, offsetAccY, offsetAccZ;
extern bool startMotor;
extern int flightMode;

void Blink()
{
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
}

void SafteyInitialization()
{
    pinMode(led, OUTPUT);

    // Make sure quadcopter is in correct starting position
    while( R[5] > 1100 )
    {
        Serial.println("Move SWB UP");
        Blink();
    }

    while( R[3] > 1100)
    {
        Serial.println("Move throttle to low position");
        Blink();
    }

    while( R[1] < 990 || R[2] < 990 || R[3] < 990 || R[4] < 990 || R[5] < 990 || R[6] < 990)
    {
        Serial.println("Turn Reciever on");
        Blink();
    }

    Serial.print("Setup Finished!");
    digitalWrite(led, HIGH);
	delay(1000);
}