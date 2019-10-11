//-------------------------------------------------
// DETERMINE THE DESIRED ATTITUDE OF THE QUADCOPTER 
//-------------------------------------------------

// GLOBAL VARIABLES
extern float actualPitch, actualRoll, actualYaw;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern int escPulse1, escPulse2, escPulse3, escPulse4;
extern float offsetPitchRate, offsetRollRate, offsetYawRate; 
extern volatile int R[7];
extern int pitchPulse, rollPulse, yawPulse;
extern float accelerationX, accelerationY, accelerationZ, averageZ, magAcceleration;
extern float offsetAccX, offsetAccY, offsetAccZ;
extern bool startMotor;
extern int flightMode;

//Outputs
int desiredPitchRate = 0, desiredRollRate = 0, desiredYawRate = 0;


void GetDesiredAttitude()
{
    int deadBand = 10;
    // ESC mid with need to be adjust when the trims of the quadcopter are changed 
    int escMid[5] = {1500, 1500, 1500, 1500, 1500};


    if( R[2] > (escMid[2] + deadBand) )
        desiredPitchRate = (escMid[2] + deadBand) - R[2];
    else if (R[2] < (escMid[2] - deadBand) )
        desiredPitchRate = (escMid[2] - deadBand) - R[2];
    else 
        desiredPitchRate = 0;
    
    if( R[1] > (escMid[1] + deadBand) )
        desiredRollRate = R[1] - (escMid[1] + deadBand);
    else if( R[1] < (escMid[1] - deadBand) )
        desiredRollRate = R[1] - (escMid[1] - deadBand);
    else 
        desiredRollRate = 0 ;

    if( R[4] > (escMid[4] + deadBand) ) 
        desiredYawRate = R[4] - (escMid[4] + deadBand);
     else if( R[4] < (escMid[4] - deadBand) )
        desiredYawRate = R[4] - (escMid[4] - deadBand);
    else 
        desiredYawRate = 0 ;

    // AutoLevel
    int autoPitch = 15 * actualPitch;
    int autoRoll = 15 * actualRoll;

    desiredPitchRate -= autoPitch;
    desiredRollRate -= autoRoll;

    desiredPitchRate /= 3;
    desiredRollRate /= 3; 
    desiredYawRate /= 3;  

}

