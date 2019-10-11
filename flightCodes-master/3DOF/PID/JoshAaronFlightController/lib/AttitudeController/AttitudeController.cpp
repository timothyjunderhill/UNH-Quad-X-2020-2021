//-----------------------------------------------------------------------------------------
// THIS SUBROUTINE TAKES THE DESIRED AND ACTUAL RATE TO DETERMINE THE ROTOR PULSE USING PID
// THE PULSE OUTPUT IS THEN BOUNDED BY A MIN AND MAX VALUE 
//------------------------------------------------------------------------------------------
// Global Variables 
extern int desiredPitchRate, desiredRollRate, desiredYawRate;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern volatile unsigned int R[7];
extern float actualPitch, actualRoll, actualYaw;

// Variables
int escPulse1 = 0, escPulse2 = 0, escPulse3 = 0, escPulse4 = 0;

//Receiver Settings//
#define escMid 1500.0	  //Middle Receiver Output Pulse [us]
#define freq 250.0 	  	  //Loop Frequency [Hz]

//Receiver Signals//
float pitchDes, rollDes, yawDes; //Desired Angles [deg]
float pitchDesD, rollDesD, yawDesD; //Desired Rates [deg/s]

//Proportional Gains
#define KpPitch 1.0
#define KpRoll 1.0
#define KpYaw 2.0

//Integral Gains//
#define KiPitch 1.7
#define KiRoll 1.7
#define KiYaw 0.0 //.02

//Derivative Gains//
#define KdPitch 0.0 //18.0
#define KdRoll 0.0 //18.0
#define KdYaw 6.0

//Max PID Control Actions//
#define uPitchMax 300
#define uRollMax 300
#define uYawMax 300

//Global Variables//
float ePitch, eRoll, eYaw; //Angle Error [deg]
float ePitchD, eRollD; //Angle Error [deg]
float ePitchDlast, eRollDlast; //Angle Error [deg]
float eiPitch, eiRoll, eiYaw; //Angle Error [deg]
float uPitch, uRoll, uYaw; //Control Actions [us]


void GetAttitudeController(){

yawDes += yawDesD/freq; if (yawDes < 0) yawDes += 360; if (yawDes > 360) yawDes -= 360; //Desired Yaw Angle [deg]
if (R[5] < escMid) yawDes = actualYaw;

//Prevent Jumpstart
pitchDes = 0;
rollDes = 0; 

//Angle Error [deg]//
ePitch = pitchDes - actualPitch;
eRoll = rollDes - actualRoll;
eYaw = yawDes - actualYaw;

ePitchD = desiredPitchRate - actualPitchRate;
eRollD = desiredRollRate - actualRollRate;

//Integral Error [deg]//
if (R[3] > 1100){
eiPitch += ePitch;
eiRoll += eRoll;
eiYaw += eYaw;}

//Shortest Way Home Correction//
if (eYaw > 180.0) eYaw -= 360.0;
if (eYaw < -180.0) eYaw += 360.0;

//PID Control Actions [us]//
uPitch = KpPitch*ePitchD + KdPitch*(ePitchD - ePitchDlast) + KiPitch*ePitch;
uRoll = KpRoll*eRollD + KdRoll*(eRollD - eRollDlast) + KiRoll*eRoll;
uYaw = KpYaw*eYaw + KdYaw*(desiredYawRate - actualYawRate);

eRollDlast = eRollD;
ePitchDlast = ePitchD;

//Bound Control Actions [us]//
uPitch = (uPitch > uPitchMax) ? uPitchMax : (uPitch < -uPitchMax) ? -uPitchMax : uPitch;
uRoll = (uRoll > uRollMax) ? uRollMax : (uRoll < -uRollMax) ? -uRollMax : uRoll;
uYaw = (uYaw > uYawMax) ? uYawMax : (uYaw < -uYawMax) ? -uYawMax : uYaw;

//ESC Pulses [us]//
escPulse1 = R[3] - uRoll + uPitch + uYaw;
escPulse2 = R[3] - uRoll - uPitch - uYaw;
escPulse3 = R[3] + uRoll - uPitch + uYaw;
escPulse4 = R[3] + uRoll + uPitch - uYaw;

}

// idle motors 
int minPulse = 1100;

// max motors
int maxPulse = 2000;

void BoundPulse()
{
	// Upper Bound 
	if (escPulse1 > maxPulse)
	{
		escPulse1 = maxPulse;
	}

	if (escPulse2 > maxPulse)
	{
		escPulse2 = maxPulse;
	}

	if (escPulse3 > maxPulse)
	{
		escPulse3 = maxPulse;
	}

	if (escPulse4 > maxPulse)
	{
		escPulse4 = maxPulse;
	}

	// LowerBound 
	if (escPulse1 < minPulse)
	{
		escPulse1 = minPulse;
	}

	if (escPulse2 < minPulse)
	{
		escPulse2 = minPulse;
	}

	if (escPulse3 < minPulse)
	{
		escPulse3 = minPulse;
	}

	if (escPulse4 < minPulse)
	{
		escPulse4 = minPulse;
	}

	// Start/Kill MOTORS
	if( R[5] < escMid)
	{
		escPulse1 = 1000;
		escPulse2 = 1000;
		escPulse3 = 1000;
		escPulse4 = 1000;

		// Reset Integral term until takeoff
		eiPitch = 0;
		eiRoll = 0;
		eiYaw = 0;
	}

	return;
}
