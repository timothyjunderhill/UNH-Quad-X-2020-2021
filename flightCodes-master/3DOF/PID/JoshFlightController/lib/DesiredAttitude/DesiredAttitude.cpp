//--------------------------------------------------------------------------
// THIS SUBROUTINE DETERMINES THE DESIRED ATTITUDE OF THE QUADCOPTER FROM 
// THE HAND HELD CONTROLLER. 
//--------------------------------------------------------------------------


// Note: the desired rates are putting out zero and nan!! need to fix!! 
// Global Variables 
extern volatile unsigned int roll_ratePulse;
extern volatile unsigned int pitch_ratePulse;
extern volatile unsigned int yaw_ratePulse;
extern float actualPitch, actualRoll;
int desiredPitchRate = 0, desiredRollRate = 0, desiredYawRate = 0;

// Local Variables
bool autoLevel = true;
int autoPitch = 0, autoRoll = 0; 
int deadbandHigh = 1508;
int deadbandLow = 1492; 

void GetDesiredAttitude()
{
	// Pitch bandwith of 16
	if(pitch_ratePulse > deadbandHigh)
		desiredPitchRate = deadbandHigh - pitch_ratePulse;
	else if(pitch_ratePulse < deadbandLow)
		desiredPitchRate = deadbandLow - pitch_ratePulse;
	else 
		desiredPitchRate = 0;

	// Roll bandwith of 16
	if(roll_ratePulse > deadbandHigh)
		desiredRollRate = roll_ratePulse - deadbandHigh;
	else if(roll_ratePulse < deadbandLow)
		desiredRollRate = roll_ratePulse - deadbandLow;
	else
		desiredRollRate = 0;

	// Yaw bandwidth of 16
	if(yaw_ratePulse > deadbandHigh)
		desiredYawRate = yaw_ratePulse - deadbandHigh;
	else if(yaw_ratePulse < deadbandLow)
		desiredYawRate = yaw_ratePulse - deadbandLow ;
	else
		desiredYawRate = 0;

	desiredYawRate /= 3;

	// AutoLevel
	autoPitch = 15*actualPitch;
	autoRoll = 15*actualRoll;

	if (autoLevel == false)
	{
		autoPitch = 0;
		autoRoll = 0;
	}

	desiredPitchRate -= autoPitch;
	desiredPitchRate /= 3; 

	desiredRollRate -= autoRoll;
	desiredRollRate /= 3;

	

	return;
}