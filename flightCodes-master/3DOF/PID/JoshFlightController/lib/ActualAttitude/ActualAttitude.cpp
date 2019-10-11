// ------------------------------------------------------------------------
// THIS SUBROUTINE DETERMINES THE ACTUAL ATTITUDE OF THE QUADCOPTER USING A 
// NXP 9DOF ADIAFRUIT IMU. BE SURE TO CALIBRATE BEFORE USE. 
// -------------------------------------------------------------------------

#include <Arduino.h> 							        //Used for Visual Studio's code (May not have to include this) 
#include <Wire.h>    							        //I2C communication 
#include <Adafruit_Sensor.h> 						    // IMU libaries 
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Mahony.h>							// Filters for IMU
//#include <Madgwick.h>

// Global Variables 
float actualPitch = 0, actualRoll = 0, actualYaw = 0;
float actualPitchRate = 0, actualRollRate = 0, actualYawRate = 0;

// Local Variables
float pitch_prev = 0, roll_prev = 0, yaw_prev = 0;
float offsetPitch_rate = 0, offsetRoll_rate = 0, offsetYaw_rate = 0;

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// Calibration of magnetometer
// Offsets applied to raw x/y/z mag values
 float mag_offsets[3] = { 28.00F, 19.49F, 55.95F };

// Soft iron error compensation matrix
 float mag_softiron_matrix[3][3] = { {  0.898,  -0.059,  0.056 },
									 {  -0.059,  1.044, -0.026 },
									 {  0.056, -0.026,  1.074 } };

 float mag_field_strength = 48.76F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
 float gyro_zero_offsets[3]  = { 0.00F, 0.00F, 0.00F };

// Filter type
// Mahnony filter is "ligher" than Madwich
 Mahony filter;
 //Madgwick filter;

 void imuIntilization()
{

	// Gyro
	while(!gyro.begin(GYRO_RANGE_250DPS))
		Serial.println("Ooops, no gyro detected ... Check your wiring!");

	// Accellerometer
	while(!accelmag.begin(ACCEL_RANGE_4G))
		Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");

	// Start IMU filter for desired frequency
	filter.begin(250);
}

void GetActualAttitude()
{
	 sensors_event_t gyro_event;
	 sensors_event_t accel_event;
	 sensors_event_t mag_event;
	 gyro.getEvent(&gyro_event);
	 accelmag.getEvent(&accel_event, &mag_event);

	 // Apply mag offset compensation (base values in uTesla)
	 float x = mag_event.magnetic.x - mag_offsets[0];
	 float y = mag_event.magnetic.y - mag_offsets[1];
	 float z = mag_event.magnetic.z - mag_offsets[2];

	 // Apply mag soft iron error compensation
	 float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
	 float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
	 float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

	 // Apply gyro zero-rate error compensation
	 float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
	 float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
	 float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

	 // The filter library expects gyro data in degrees/s, but adafruit sensor
	 // uses rad/s so we need to convert them first (or adapt the filter lib
	 // where they are being converted)
	 gx *= 57.2958F;
	 gy *= 57.2958F;
	 gz *= 57.2958F;

	filter.update(gx, gy, gz,
				  accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
				  mx, my, mz);

	 // Degrees 
	 actualPitch = filter.getRoll();
	 actualRoll = filter.getPitch();
	 actualYaw = -1*filter.getYaw(); 										// negative sign added to correct sign convention

	 // Degrees per second 
	 actualPitchRate = gyro_event.gyro.x*(180/3.14);
	 actualRollRate = gyro_event.gyro.y*(180/3.14);
	 actualYawRate = -1*gyro_event.gyro.z*(180/3.14);    // negative sign added to correct sign convention

}





