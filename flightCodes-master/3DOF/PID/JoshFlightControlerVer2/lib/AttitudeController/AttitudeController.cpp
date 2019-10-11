//-----------------------------------------------------------------------------------------
// THIS SUBROUTINE TAKES THE DESIRED AND ACTUAL RATE TO DETERMINE THE ROTOR PULSE USING PID
// THE PULSE OUTPUT IS THEN BOUNDED BY A MIN AND MAX VALUE 
//------------------------------------------------------------------------------------------

// GLOBAL VARIABLES USED FOR DEBUGGING
extern float actualPitch, actualRoll, actualYaw;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern int desiredPitchRate, desiredRollRate, desiredYawRate;
extern float offsetPitchRate, offsetRollRate, offsetYawRate; 
extern int escPulse1, escPulse2, escPulse3, escPulse4;
extern volatile int R[7];
extern float accelerationX, accelerationY, accelerationZ, averageZ, magAcceleration;
extern float offsetAccX, offsetAccY, offsetAccZ;
extern bool startMotor;
extern int flightMode;

// Variables


// Pitch 
float errorPitch = 0, errorRoll = 0, errorYaw = 0;
int pitchPulse = 0, rollPulse = 0, yawPulse = 0;
float last_errorPitch = 0, last_errorRoll = 0, last_errorYaw = 0;
float Ipitch = 0, Iroll = 0, Iyaw = 0;


// Gains
const float pPitch = 1.3;
const float dPitch = 18.0;
const float iPitch = 0.00;
const float pid_max_pitch = 100;

const float pRoll = pPitch;
const float dRoll = dPitch;
const float iRoll = iPitch;
const float pid_max_roll = 100;

const float pYaw = 4.0;
const float dYaw = 0.0;
const float iYaw = 0.00;
const float pid_max_yaw = 100;

void GetAttitudeController()
{

	// Pitch
	errorPitch = desiredPitchRate - actualPitchRate;
	Ipitch += iPitch*errorPitch; 

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

	return;

}
