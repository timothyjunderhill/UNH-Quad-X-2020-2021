

// GLOBAL VARIABLES USED FOR DEBUGGING
extern float actualPitch, actualRoll, actualYaw;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern int desiredPitchRate, desiredRollRate, desiredYawRate;
extern int escPulse1, escPulse2, escPulse3, escPulse4;
extern volatile int R[7];
extern float offsetPitchRate, offsetRollRate, offsetYawRate; 
extern int pitchPulse, rollPulse, yawPulse;
extern float accelerationX, accelerationY, accelerationZ, averageZ, magAcceleration;
extern bool startMotor;
extern int flightMode;


// LOCAL VARIABLES 
bool startMotor = false, takeOff = false;
int flightMode = 0; 
int takeOffThrottle = 0, throttle = 1100;

void GetMode()
{
	// Determine the quadstate
	if( R[5] > 1500)
	{
		startMotor = true;
	}
	else
	{
		startMotor = false;
		takeOff = false;
		takeOffThrottle = 0;
	}

	// FlightMode 
	if( R[6] < 1200) // manual 
	{
		flightMode = 1;
	}
	else if(R[6] > 1200 && R[6] < 1700) // auto takeOff 
	{
		flightMode = 2;
	}
	else // auto landing 
	{
		flightMode = 3;
	}

	// actions of different flightmodes
	// NOTE: NEED TO ADD A WAY TO REMOVE THE CORRECTIONS FROM THE ATTITUDE CONTROLLER ON TAKEOFF
	if(startMotor == true && flightMode == 2 && takeOff == false) // autotake off 
	{
		if( R[3] > 1400 && R[3] < 1600) // center throttle stick 
		{
			throttle++;
			if(throttle > 1750) // safety 
			{
				takeOffThrottle = 0;
				takeOff = true;
			}
			if(averageZ > 10.40) // detected takeOff
			{
				takeOff = true;
				takeOffThrottle = throttle - 1530;
			}

		}
	}

	if(startMotor == true && flightMode == 3 && takeOff == true) // autoLanding 
	{
		if(throttle > 1100 )
		{
			throttle--;
		}
		if(throttle <= 1100 )
		{
			takeOff == false;
		}
		
	}

	// Throttle Setting for different flightmodes
	if(startMotor == true && flightMode == 2 && takeOff == true) // Throttle after autotakeOff 
	{
		throttle = R[3] + takeOffThrottle;
	}

	if(startMotor == true && flightMode == 1) // manual mode
	{
		throttle = R[3];
	}


}



