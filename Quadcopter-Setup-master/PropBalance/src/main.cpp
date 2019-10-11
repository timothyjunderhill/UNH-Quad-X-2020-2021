#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Mahony.h>
#include <Madgwick.h>

elapsedMicros elapsedTime;

// Attitude loop timing 
int updateTime = 4000;
long lastUpdate = 0;

// Printing loop timing 
long lastPrint = 0;
int printTimer = 5000;


Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

extern float offsetPitchRate, offsetRollRate, offsetYawRate; 
extern float offsetAccX, offsetAccY, offsetAccZ;

//------------------------
// IMU CALIBRATION VALUES
//------------------------

// Offsets applied to raw x/y/z mag values
float mag_offsets[3] = { -20.02F, -26.84F, 48.86F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.953,  -0.0019,  0.009 },
									 {  -0.059,  1.022, -0.024 },
									 {  0.009, -0.024,  1.028  } };

float mag_field_strength = 48.75F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Filter sample rate
int updateFreq = 250;

// Filter type
//Mahony filter;
Madgwick filter;

// Global variables 
float actualPitch = 0, actualRoll = 0, actualYaw = 0;
float actualPitchRate = 0, actualRollRate = 0, actualYawRate = 0;
float accelerationX = 0, accelerationY = 0, accelerationZ = 0;
float magAcceleration = 0;

// Variables for filter 
float averageZ = 0, sumZ = 0;
float storage[25] = {};
int counter = 0;

void IMUInitialization()
{
	if(!gyro.begin(GYRO_RANGE_250DPS))
	{
		Serial.println("Ooops, no gyro detected ... Check your wiring!");
		while(1);
	}

	if(!accelmag.begin(ACCEL_RANGE_4G))
	{
		Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
		while(1);
	}

	filter.begin(updateFreq);
	Serial.println("IMU initialization Completed");
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

	actualRoll = filter.getPitch();
	actualPitch = filter.getRoll();
	actualYaw = -1*filter.getYaw();

	actualPitchRate =  gx; 
	actualRollRate = gy;
	actualYawRate = -1*gz; 

	accelerationX = -accel_event.acceleration.y;
	accelerationY = -accel_event.acceleration.x; 
	accelerationZ = accel_event.acceleration.z; 

	magAcceleration = sqrt(accelerationX*accelerationX + accelerationY*accelerationY + accelerationZ*accelerationZ);

	 // Average the acceleration data 
	//  sumZ += accelerationZ;
	//  sumZ -= storage[counter];
	//  averageZ = sumZ/25;
	//  storage[counter] = accelerationZ;
	//  if(counter == 24)
	//  	counter = 0;
	//  else
	//  	counter++;
}

int spinMotor = 10;    // which motor will spin
int prevSpinMotor = 10; 

//Receiver Pin
const int ch3 = 26; 		//Left Stick - Vertical (Throttle)

// Interrupt  Variables 
elapsedMicros time;  
volatile int R = 0;
unsigned long timer = 0;

// ESC's 
// Get maximum value for selected PWM resolution (100% Duty)
int pwmMax = 256;
// Initializing pulse for ESCs, 25% duty
int escInit = pwmMax/4;
int pwmFreq = 250;
int pwmRes = 8;
int escPulseTime = 4000;

// esc pins
 int escOut1 = 6;
 int escOut2 = 10;
 int escOut3 = 5;
 int escOut4 = 20;

// Pulse length 
int escPulse1 = 0;
int escPulse2 = 0;
int escPulse3 = 0;
int escPulse4 = 0;

// Value for analog write function 
int escPulse1PWM = 0;
int escPulse2PWM = 0;
int escPulse3PWM = 0;
int escPulse4PWM = 0;

void EscInitialize()
{
	// All on Timer FTM0 -> pwmFreq
  	analogWriteFrequency(escOut1, pwmFreq);

  	// Set PWM resolution
  	analogWriteResolution(pwmRes);
	
	// Initialize ESCs
  	analogWrite(escOut1, escInit);
  	analogWrite(escOut2, escInit);
  	analogWrite(escOut3, escInit);
  	analogWrite(escOut4, escInit);
  	delay(5000);
	Serial.println("ESC Initialization Completed");	
}

// Read the Input value from the user 
void ReadInput()
{
  if(Serial.available() > 0)                          // make sure input is available 
  {
    char inChar = Serial.read();  
    if(inChar >= '0' && inChar <= '4')                      
		{
      prevSpinMotor = spinMotor;
			spinMotor = (inChar - '0');                        // Convert the ascii to number and add to value
		}
		else                                                    // Typed in some other character, restart 
		{
			Serial.println("Please input a number 1 through 4");          
			spinMotor = 0;                                     // Reset the spinMotor                    
		}
    if( prevSpinMotor != spinMotor)
    {
      Serial.print("Spin Motor: ");
      Serial.println(spinMotor);
      delay(500);
    }
  }
}

// Determine which motor to spin
void SpinMotor()
{
  switch(spinMotor) {
    case 1:
      escPulse1 = R;
      escPulse2 = 1000;
      escPulse3 = 1000;
      escPulse4 = 1000;
      break;
    case 2:
      escPulse1 = 1000;
      escPulse2 = R;
      escPulse3 = 1000;
      escPulse4 = 1000;
      break;
    case 3:
      escPulse1 = 1000;
      escPulse2 = 1000;
      escPulse3 = R;
      escPulse4 = 1000;
      break;
    case 4: 
      escPulse1 = 1000;
      escPulse2 = 1000;
      escPulse3 = 1000;
      escPulse4 = R;
      break;
    default:
      escPulse1 = 1000;
      escPulse2 = 1000;
      escPulse3 = 1000;
      escPulse4 = 1000;
  }
}

// Convert the Pulse length to a PWM signal 
void pulsetoPWM()
{
	// Convert Mircosecond time to PWM pulse for motors(CREATE NEW FUNCTION WILL NEXT LINES)
	escPulse1PWM = escPulse1*pwmMax/escPulseTime;
	escPulse2PWM = escPulse2*pwmMax/escPulseTime;
	escPulse3PWM = escPulse3*pwmMax/escPulseTime;
	escPulse4PWM = escPulse4*pwmMax/escPulseTime;

	// Send PWM pulse to motors
	analogWrite(escOut1, escPulse1PWM);
  analogWrite(escOut2, escPulse2PWM);
  analogWrite(escOut3, escPulse3PWM);
  analogWrite(escOut4, escPulse4PWM);
}

//External Interrupt to record pulse length 
void ch3Int(){ 	if (digitalReadFast(ch3)) timer = time;  else R = time - timer;}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
	while(!Serial);


	EscInitialize();                    // Initialize the ESCs 
	attachInterrupt(ch3,ch3Int,CHANGE); // Interrupt initialization 
  IMUInitialization();                // Start IMU 

  Serial.println("BALANCE PROPS");
	Serial.println("SELECT 1-4 TO SPIN DESIRED MOTOR");
  Serial.println("PUT TAPE ON PROPELLER BLADES TO REDUCE VIBRATIONS ");
  delay(4000);
}

void loop() {
  if ((elapsedTime - lastUpdate) > updateTime)
  {
    GetActualAttitude();
    ReadInput();
    SpinMotor();
    pulsetoPWM();
  }
  if((elapsedTime - lastPrint) >= printTimer)
  {
    Serial.println(magAcceleration);
  }
}