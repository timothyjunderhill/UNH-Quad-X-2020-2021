//-----------------------------------------------------------------------------------------
// THIS SUBROUTINE TAKES THE DESIRED AND ACTUAL RATE TO DETERMINE THE ROTOR PULSE USING PID
// THE PULSE OUTPUT IS THEN BOUNDED BY A MIN AND MAX VALUE 
//------------------------------------------------------------------------------------------
// Global Variables 
extern int desiredPitchRate, desiredRollRate, desiredYawRate;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern volatile unsigned int throttle_Pulse, activateMotor;


// Variables
int escPulse1 = 0, escPulse2 = 0, escPulse3 = 0, escPulse4 = 0;

// Pitch 
float errorPitch = 0, errorRoll = 0, errorYaw = 0;
int pitchPulse = 0, rollPulse = 0, yawPulse = 0;
float last_errorPitch = 0, last_errorRoll = 0, last_errorYaw = 0;
float Ipitch = 0, Iroll = 0, Iyaw = 0;


// Gains
float pPitch = 1.3;
float dPitch = 18.0;
float iPitch = 0;
float pid_max_pitch = 100;

float pRoll = pPitch;
float dRoll = dPitch;
float iRoll = iPitch;
float pid_max_roll = 100;

float pYaw = 4.0;
float dYaw = 0.0;
float iYaw = 0.02;
float pid_max_yaw = 100;

void GetAttitudeController()
{

	// Pitch
	errorPitch = desiredPitchRate - actualPitchRate;
	Ipitch += iPitch*errorPitch; 
	if(Ipitch > 10)
		Ipitch = 10;
	else if(Ipitch < -10)
		Ipitch = -10;

	pitchPulse = pPitch*errorPitch + dPitch*(errorPitch - last_errorPitch) + Ipitch;
	last_errorPitch = errorPitch; 

	// Bound PID Pitch output
	if( pitchPulse > pid_max_pitch)
	{
		pitchPulse = pid_max_pitch;
	}

	if( pitchPulse < -pid_max_pitch )
	{
		pitchPulse = -pid_max_pitch;
	}


	// Roll 
	errorRoll = desiredRollRate - actualRollRate;
	Iroll += iRoll*errorRoll;
	if(Iroll > 10)
		Iroll = 10;
	else if(Iroll < -10)
		Iroll = -10;
	
	rollPulse = pRoll*errorRoll + dRoll*(errorRoll - last_errorRoll) + Iroll;
	last_errorRoll = errorRoll;

	// Bound PID Roll output 
	if( rollPulse > pid_max_roll)
	{
		rollPulse = pid_max_roll;
	}

	if( rollPulse < -pid_max_roll )
	{
		rollPulse = -pid_max_roll;
	}


	// Yaw
	errorYaw = desiredYawRate - actualYawRate;
	Iyaw += iYaw*errorYaw;
	if(Iyaw > 10)
		Iyaw = 10;
	else if(Iyaw < -10)
		Iyaw = -10;
	yawPulse = pYaw*errorYaw +dYaw*(errorYaw - last_errorYaw) + Iyaw;
	last_errorYaw = errorYaw;

	// Bound PID YAW output  
	if( yawPulse > pid_max_yaw)
	{
		yawPulse = pid_max_yaw;
	}

	if( yawPulse < -pid_max_yaw )
	{
		yawPulse = -pid_max_yaw;
	}

	// Calculate pulses to motors
	escPulse1 = throttle_Pulse - rollPulse + pitchPulse + yawPulse;
	escPulse2 = throttle_Pulse - rollPulse - pitchPulse - yawPulse;
	escPulse3 = throttle_Pulse + rollPulse - pitchPulse + yawPulse; 
	escPulse4 = throttle_Pulse + rollPulse + pitchPulse - yawPulse;

	return;

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
	if(activateMotor < 1100)
	{
		escPulse1 = 1000;
		escPulse2 = 1000;
		escPulse3 = 1000;
		escPulse4 = 1000;
		// Ipitch = 0;
		// Iroll = 0;
		// Iyaw = 0;
	}

	return;
}
