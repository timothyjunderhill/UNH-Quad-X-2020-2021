//-------------------------------------------------
// DETERMINE THE DESIRED ATTITUDE OF THE QUADCOPTER 
//-------------------------------------------------
#include <Arduino.h>
//Inputs 
extern volatile int R[7];
extern float actualPitch, actualRoll;

#define maxPitch 164.0    //Max Desired Pitch [deg/s]
#define maxRoll 164.0     //Max Desired Roll [deg/S]
#define maxYaw 64 		  //Max Desired Yaw Rate [deg/s]

#define pulseScale 500.0  //Pulse Full Scale [us]

#define escMid 1500.0	  //Middle Receiver Output Pulse [us]

#define deadBand 10       //Controller Performance Deadband [us]

//Outputs
int desiredPitchRate = 0, desiredRollRate = 0, desiredYawRate = 0;


void GetDesiredAttitude()
{

    //Figure out these three
    desiredPitchRate = (abs(R[2] - escMid) < deadBand) ? 0 : -(R[2] - escMid)*maxPitch/pulseScale; //Desired Pitch Angle [deg]
    desiredRollRate= (abs(R[1] - escMid) < deadBand) ? 0 : (R[1] - escMid)*maxRoll/pulseScale;     //Desired Roll Angle [deg]
    desiredYawRate = (abs(R[4] - escMid) < deadBand) ? 0 : -(R[4] - escMid)*maxYaw/pulseScale;     //Desired Yaw Rate [deg/s]

}

