//-----------------------------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------- Quadcopter Flight Code ---------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------- Created by Aaron Cantara at the University of New Hampshire 2019 ------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------------------//

//Libraries//
#include <Arduino.h> 				      //Visual Studio's Arduino Extension
#include <Adafruit_Sensor.h> 		  //Adafruit Sensors
#include <Adafruit_FXAS21002C.h>  //FXAS21002C Gyroscope (Gyro)
#include <Adafruit_FXOS8700.h> 		//FXOS8700 Accelerometer and Magnetometer (Mag)
#include <Madgwick.h> 			    	//Madgwick Attitude and Heading Reference System (AHRS)

//-----------------------------------------------------------------------------------------------------------------------------------------//
//INERTIAL MEASUREMENT UNIT (IMU)
//-----------------------------------------------------------------------------------------------------------------------------------------//

//Mag Bias Calibration [uT]//
#define magCalX -7.98
#define magCalY -18.24   
#define magCalZ -29.89

//Gyro Bias Calibration [deg/s]//
#define gyroCalX -0.404721467
#define gyroCalY 0.899240366
#define gyroCalZ 0.40121665

#define rad2deg 180.0/PI

//Angular Kinematics//
float pitch, roll, yaw;     //Madgwick Euler Angles [deg]
float pitchD, rollD, yawD;  //Gyro Body Angular Rates [deg/s]

//Magnetometer Calibration//
float mag_field_strength = 37.88F;                           //Magnetic Field Strength [uT]
float softIron[3][3] = {{ 0.990, -0.033,  -0.017  },         //Soft Iron Error Compensation Matrix
                       { -0.033,  0.977,   0.013  },
                       { -0.017,  0.013,   1.035 }};

//Sensors and Filtering//
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);        //Gyroscope Model 
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);  //Accelerometer and Magnetometer Model 
Madgwick filter;                                                   //Madgwick AHRS Euler Angle Filter

void imu(){

  //Communication//
  sensors_event_t gyro_event; sensors_event_t accel_event; sensors_event_t mag_event;
  gyro.getEvent(&gyro_event); accelmag.getEvent(&accel_event, &mag_event);

  //Raw Magnetometer Values [uT]//
  float x = mag_event.magnetic.x - magCalX;
  float y = mag_event.magnetic.y - magCalY;
  float z = mag_event.magnetic.z - magCalZ;

  //Soft Iron Error Compensation//
  float mx = x * softIron[0][0] + y * softIron[0][1] + z * softIron[0][2];
  float my = x * softIron[1][0] + y * softIron[1][1] + z * softIron[1][2];
  float mz = x * softIron[2][0] + y * softIron[2][1] + z * softIron[2][2];

  //Gyro Angular Rates [deg/s]//
  rollD = -gyro_event.gyro.x*rad2deg - gyroCalX;
  pitchD = gyro_event.gyro.y*rad2deg - gyroCalY;
  yawD = gyro_event.gyro.z*rad2deg - gyroCalZ;

  //Update Angle Filter//
  filter.update(-rollD, pitchD, yawD, accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z, mx, my, mz);

  //Euler Angles [deg]//
  roll = -filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();
}

//-----------------------------------------------------------------------------------------------------------------------------------------//
//RECEIVER PIN CHANGE INTERRUPTS
//-----------------------------------------------------------------------------------------------------------------------------------------//

//Receiver Pins//
#define ch1 24  		//Right Stick - Horizontal (Roll)
#define ch2 25			//Right Stick - Vertical (Pitch)
#define ch3 26  		//Left Stick - Vertical (Throttle)
#define ch4 27			//Left Stick - Horizontal (Yaw)
#define ch5 28  		//Left Nob (Start)
#define ch6 29  		//Right Nob

//Receiver Settings//
#define deadBand 10 	    //Controller Performance Deadband [us]
#define pulseScale 500.0  //Pulse Full Scale [us]
#define maxPitch 164.0    //Max Desired Pitch [deg/s]
#define maxRoll 164.0     //Max Desired Roll [deg/S]
#define maxYaw 64         //Max Desired Yaw Rate [deg/s]
#define escMid 1500	      //Middle Receiver Output Pulse [us]
#define freq 250.0 	      //Loop Frequency [Hz]

//Receiver Signals//
elapsedMicros time;   			        //Clock [us]
unsigned long timer[7]; 	          //Timer [us]
volatile int R[7] = {0};	          //Hand-Held Receiver Signals [us]
float pitchDes, rollDes, yawDes;    //Desired Angles [deg]
float pitchDesD, rollDesD, yawDesD; //Desired Rates [deg/s]

//External Interrupts//
void ch1Int(){  if (digitalReadFast(ch1)) timer[1] = time;  else R[1] = time - timer[1];}
void ch2Int(){ 	if (digitalReadFast(ch2)) timer[2] = time;  else R[2] = time - timer[2];}
void ch3Int(){ 	if (digitalReadFast(ch3)) timer[3] = time;  else R[3] = time - timer[3];}
void ch4Int(){  if (digitalReadFast(ch4)) timer[4] = time;  else R[4] = time - timer[4];}
void ch5Int(){ 	if (digitalReadFast(ch5)) timer[5] = time;  else R[5] = time - timer[5];}
void ch6Int(){  if (digitalReadFast(ch6)) timer[6] = time;	else R[6] = time - timer[6];}

//Receiver Control Inputs//
void receiver(){

  pitchDesD = (abs(R[2] - escMid) < deadBand) ? 0 : -(R[2] - escMid)*maxPitch/pulseScale;     //Desired Pitch Angle [deg]
  rollDesD = (abs(R[1] - escMid) < deadBand) ? 0 : (R[1] - escMid)*maxRoll/pulseScale;        //Desired Roll Angle [deg]
  yawDesD = (abs(R[4] - escMid) < deadBand) ? 0 : -(R[4] - escMid)*maxYaw/pulseScale;        //Desired Yaw Rate [deg/s]
  yawDes += yawDesD/freq;  if (yawDes < 0) yawDes += 360;  if (yawDes > 360) yawDes -= 360;  //Desired Yaw Angle [deg]
  if (R[5] < escMid) yawDes = yaw;   
  
  pitchDes = 0;
  rollDes = 0;                                                        //Prevent Jumpstart

}
	 
//-----------------------------------------------------------------------------------------------------------------------------------------//
//PID ATTITUDE CONTROLLER
//-----------------------------------------------------------------------------------------------------------------------------------------//

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
float ePitch, eRoll, eYaw; 		     //Angle Error [deg]
float ePitchD, eRollD; 		         //Angle Error [deg]
float ePitchDlast, eRollDlast; 		 //Angle Error [deg]
float eiPitch, eiRoll, eiYaw; 		 //Angle Error [deg]
float uPitch, uRoll, uYaw;		     //Control Actions [us]
float save[3];				   	         //Previous Loop Values
float esc[5]; 					           //ESC Pulse Length [us] 
 
void attitude(){

  //Angle Error [deg]//
  ePitch = pitchDes - pitch;
  eRoll = rollDes - roll;
  eYaw = yawDes - yaw;

  ePitchD = pitchDesD - pitchD;
  eRollD = rollDesD - rollD;

  //Integral Error [deg]//
  if (R[3] > 1100){
  eiPitch += ePitch;
  eiRoll += eRoll;
  eiYaw += eYaw;}

  if (R[5] < escMid) { eiPitch = 0;  eiRoll = 0; eiYaw = 0; }  
 
  //Shortest Way Home Correction//
  if (eYaw > 180.0) eYaw -= 360.0;  
  if (eYaw < -180.0) eYaw += 360.0;
 
  //PID Control Actions [us]//
  uPitch = KpPitch*ePitchD + KdPitch*(ePitchD - ePitchDlast) + KiPitch*ePitch;
  uRoll = KpRoll*eRollD + KdRoll*(eRollD - eRollDlast) + KiRoll*eRoll;
  uYaw = KpYaw*eYaw + KdYaw*(yawDesD - yawD);

  eRollDlast = eRollD;
  ePitchDlast = ePitchD;
	
  //Bound Control Actions [us]//
  uPitch = (uPitch > uPitchMax) ? uPitchMax : (uPitch < -uPitchMax) ? -uPitchMax : uPitch;
  uRoll = (uRoll > uRollMax) ? uRollMax : (uRoll < -uRollMax) ? -uRollMax : uRoll; 
  uYaw = (uYaw > uYawMax) ? uYawMax : (uYaw < -uYawMax) ? -uYawMax : uYaw;
}

//-----------------------------------------------------------------------------------------------------------------------------------------//
//Battery Voltage Monitoring and Compensation
//-----------------------------------------------------------------------------------------------------------------------------------------//

#define led 13 		   //Warning LED Pin
#define batt 32      //Battery Monitor Pin
#define N 1000
#define battComp 40.0 
#define battSlope 0.013279114088
#define battIntercept 0.091917237806
#define lowBatt 11.1

float voltage = 12.6;
int count = 0, sum = 0;

//Measure Battery Voltage//
void battery(){

  //Battery Voltage Monitoring//
  sum += analogRead(batt);
  if (++count == N) {voltage = battSlope*sum/count + battIntercept; count = 0;  sum = 0;} //Moving Average Filter
  if (voltage < lowBatt) digitalWrite(led,HIGH);}        //Battery Low -> Warning LED On

//-----------------------------------------------------------------------------------------------------------------------------------------//
//Electronic Speed Controllers (ESCs)
//-----------------------------------------------------------------------------------------------------------------------------------------//

bool start = false;  //Motors On/Off Switch

//Pins//
#define escPin1 6 	 //Front Left (cw)
#define escPin2 10   //Front Right (ccw)
#define escPin3 5 	 //Back Right (cw)
#define escPin4 20 	 //Back Left (cw)

//Settings//
#define escOff 1000     //Motor Off Pulse Length [us]
#define escMin 1100     //Min Motor Pulse Length [us]
#define escMax 2000     //Max Motor Pulse Length [us]
#define pwmMax 256		  //Max PWM
#define period 4000		  //Loop Period [us]

void escWrite(){

  //ESC Pulses [us]//
  esc[1] = R[3] + uRoll + uPitch - uYaw;
  esc[2] = R[3] - uRoll + uPitch + uYaw;
  esc[3] = R[3] - uRoll - uPitch - uYaw; 
  esc[4] = R[3] + uRoll - uPitch + uYaw;

  //Battery Loss Compensation//
  if (voltage < 12.40 && voltage > 6.0) {   //Battery Connected?
    esc[1] += (12.40 - voltage)*battComp; esc[2] += (12.40 - voltage)*battComp;
    esc[3] += (12.40 - voltage)*battComp; esc[4] += (12.40 - voltage)*battComp;}

  //Bound ESC Pulses//
  esc[1] = (esc[1] > escMax) ? escMax : (esc[1] < escMin) ? escMin : esc[1];
  esc[2] = (esc[2] > escMax) ? escMax : (esc[2] < escMin) ? escMin : esc[2];
  esc[3] = (esc[3] > escMax) ? escMax : (esc[3] < escMin) ? escMin : esc[3];
  esc[4] = (esc[4] > escMax) ? escMax : (esc[4] < escMin) ? escMin : esc[4];

  //Motors On/Off Switch//
  start = (R[5] < escMid) ? false : (R[5] > escMid && R[3] < 1150.0) ? true : start;   //Switch
  if (!start) {esc[1] = escOff; esc[2] = escOff;  esc[3] = escOff;  esc[4] = escOff;}  //Motors Off

	//Send PWM to ESCs//
  analogWrite(escPin1, esc[1]*pwmMax/period);	analogWrite(escPin2, esc[2]*pwmMax/period);
  analogWrite(escPin3, esc[3]*pwmMax/period); analogWrite(escPin4, esc[4]*pwmMax/period);
}

//-----------------------------------------------------------------------------------------------------------------------------------------//
//SETUP
//-----------------------------------------------------------------------------------------------------------------------------------------//

//Misc Constants//
#define escInit 64   //Initialize Pulse (25% duty)
#define pwmRes 8	   //PWM Resolution

void setup() {

  //Setup Begin//
  pinMode(led,OUTPUT); digitalWrite(led,HIGH); //Warning LED On
  Serial.begin(115200);                        //Serial Monitor

  //Pin Modes//
  pinMode(escPin1,OUTPUT); pinMode(escPin2,OUTPUT); pinMode(escPin3,OUTPUT); pinMode(escPin4,OUTPUT); 			   //ESCs
  attachInterrupt(ch1,ch1Int,CHANGE);	attachInterrupt(ch2,ch2Int,CHANGE); attachInterrupt(ch3,ch3Int,CHANGE);  //Receiver Channels 1-3
  attachInterrupt(ch4,ch4Int,CHANGE);	attachInterrupt(ch5,ch5Int,CHANGE); attachInterrupt(ch6,ch6Int,CHANGE);  //Receiver Channels 4-6

  //Receiver Safety Check//
  while(R[3] == 0 || R[3] > 1100) Serial.println("Turn the receiver on and the throttle low.");
  while(R[5] > escMid) Serial.println("Turn the ignition off. It is the top leftmost receiver switch.");

  //Initialize IMU//
  while(!gyro.begin(GYRO_RANGE_500DPS)) Serial.println("Gyro not detected.");
  while(!accelmag.begin(ACCEL_RANGE_4G)) Serial.println("Accelerometer/Magnetometer not detected.");
  filter.begin(freq);   //IMU Filter Frequency

  //Initialize ESCs//
  analogWriteFrequency(escPin1, freq);  //PWM Frequency
  analogWriteResolution(pwmRes); 		    //PWM Resolution
  analogWrite(escPin1, escInit); analogWrite(escPin2, escInit); analogWrite(escPin3, escInit); analogWrite(escPin4, escInit);
  delay(5000); Serial.println("ESC Initialization Completed");	

  //Setup Complete//
  digitalWrite(led,LOW); //Warning LED Off
  timer[0] = time;
}

//-----------------------------------------------------------------------------------------------------------------------------------------//
//MAIN LOOP
//-----------------------------------------------------------------------------------------------------------------------------------------//

void loop(){
	
  //Functions//
  imu();       //Determine the quadcopter's orientation
  receiver();  //Retrieve user commands from the receiver
  attitude();  //Calculate control actions for the quadcopter's orientation
  battery();   //Monitor the battery voltage
  escWrite();  //Execute control actions by writing to the ESCs

  //Loop Timing//
  if (time - timer[0] > period) digitalWrite(led,HIGH); //Warning LED
  while(time - timer[0] < period);  timer[0] = time;    //Loop Timer
} 