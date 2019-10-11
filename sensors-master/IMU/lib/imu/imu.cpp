#include <Adafruit_Sensor.h> 				
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Madgwick.h>

 // Filter sample rate
const int updateFreq = 250;

//Mag Bias Calibration [uT]//
const float magCalX = -7.98;
const float magCalY = -18.24;   
const float magCalZ = -29.89;

//Gyro Bias Calibration [deg/s]//
const double gyroCalX = -0.404721467;
const double gyroCalY = 0.899240366;
const double gyroCalZ = 0.40121665;

const float rad2deg = 180.0/PI;

//Angular Kinematics//
float pitch = 0, roll = 0, yaw = 0;     //Madgwick Euler Angles [deg]
float pitchD = 0, rollD = 0, yawD = 0;  //Gyro Body Angular Rates [deg/s]

//Magnetometer Calibration//
float mag_field_strength = 37.88F;                           //Magnetic Field Strength [uT]
float softIron[3][3] = {{ 0.990, -0.033,  -0.017  },         //Soft Iron Error Compensation Matrix
			   { -0.033,  0.977,   0.013  },
			   { -0.017,  0.013,   1.035 }};

//Sensors and Filtering//
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);        //Gyroscope Model 
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);  //Accelerometer and Magnetometer Model 
Madgwick filter;                                                   //Madgwick AHRS Euler Angle Filter

void ImuIntilization()
{

	// Gyro
	while(!gyro.begin(GYRO_RANGE_500DPS))
		Serial.println("Incorrect Wiring");

	// Accellerometer
	while(!accelmag.begin(ACCEL_RANGE_4G))
		Serial.println("Incorrect Wiring");

	// Start IMU filter for desired frequency
	filter.begin(updateFreq);

	return;

}

 void GetIMU()
 {

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

	return; 
}