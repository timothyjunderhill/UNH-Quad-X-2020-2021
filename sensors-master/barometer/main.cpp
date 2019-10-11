/*  Teensy uses SCL0 and SDA0 pins by default. The wiring diagram based on the wires I have in my barometer is:
  GND (black) to GND
  Vin (red) to 3.3V
  SCL (blue) to pin 19 (SCL0)
  SDA (white) to pin 18 (SDA0)
  */

#include <Arduino.h>
#include <i2c_t3.h>   //Teensy Wire Library

//I2C Addresses//
#define MS5611 119            //MS5611 Barometer
#define call 0        //General Call
#define requestTemperature 88        //
#define requestPressure 72 

   
uint8_t start, error;
int16_t count_var;
uint32_t loop_timer;

#define seaLevelPressure 101325.0  


//Pressure variables.
uint16_t C[7];
uint8_t baroCount, tempCount, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;
double altitude, referenceAltitude;

//Altitude PID variables
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;


//******************************************************************************************************************************************************


//-----------------------------------------------------------------------------------------------------------------------------------------//
//PID ATTITUDE CONTROLLER
//-----------------------------------------------------------------------------------------------------------------------------------------//


void altimeter(void) {
  baroCount ++;
  
  //Every time this function is called the baroCount variable is incremented. This way a specific action
  //is executed at the correct moment. This is needed because requesting data from the MS5611 takes around 9ms to complete.

  if (baroCount == 1) {                                                 //When the baroCount variable is 1.
    if (tempCount == 0) {                                             //And the temperature counter is 0.
      //Get temperature data from MS-5611
      Wire.beginTransmission(MS5611);                                   //Open a connection with the MS5611
      Wire.write(call);                                               //Send a 0 to indicate that we want to call the requested data.
      Wire.endTransmission();                                                   //End the transmission with the MS5611.

      Wire.requestFrom(MS5611, 3);                                       //call 3 data bytes from the MS5611.
      delayMicroseconds(100);
      // Store the temperature in a 5 location rotating memory to prevent temperature spikes.
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      
      raw_temperature_rotating_memory[average_temperature_mem_location] = (Wire.read() << 16 | Wire.read() << 8 | Wire.read());

      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;                      //Calculate the avarage temperature of the last 5 measurements.
    }
    else {
      //Get pressure data from MS-5611
      Wire.beginTransmission(MS5611);                                  //Open a connection with the MS5611.
      Wire.write(call);                                               //Send a 0 to indicate that we want to call the requested data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
 
      Wire.requestFrom(MS5611, 3);                                     //call 3 data bytes from the MS5611.
      delayMicroseconds(100);
      raw_pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();     //Shift the individual bytes in the correct position and add them to the raw_pressure variable.

    }
//-----------------------------------------------------------------------------------------------------------------------------------------//
    tempCount ++;                                                     //Increase the tempCount variable.
    if (tempCount == 20) {
      Wire.beginTransmission(MS5611); 
      Wire.write(requestTemperature);  
      Wire.endTransmission(); 
      tempCount = 0;}                 
    else {
      Wire.beginTransmission(MS5611); 
      Wire.write(requestPressure);  
      Wire.endTransmission();}
}   
  if (baroCount == 2) {                                                 //If the baroCount variable equals 2.
    //Calculate pressure as explained in the datasheet of the MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
    //To get a smoother pressure value we will use a 20 location rotating memory.
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                                //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.

    //To get better results we will use a complementary fillter that can be adjusted by the fast average.
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;                                                                   //The actual_pressure is used in the program for altitude calculations.
    altitude = (44330.0f * (1.0f - pow((double)actual_pressure / (double)seaLevelPressure, 0.1902949f)));
  }

  if (baroCount == 3) {                                                                               //When the barometer counter is 3
    baroCount = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.

  }
}

//*******************************************************************************************************************************************

void setup() {  
  Serial.begin(9600);
  
  //Check if the MS5611 barometer is responding.
  Wire.begin(I2C_MASTER, call, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000); //Start the I2C as master  *IMPORTANT FOR HIGH-SPEED I2C ON TEENSY*
  Wire.beginTransmission(MS5611);                      //Start communication with the MS5611.
  error = Wire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MS5611 did not responde.
    Wire.beginTransmission(MS5611);
    error = Wire.endTransmission();
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  
  //For calculating the pressure the 6 calibration values need to be called from the MS5611.
  //These 2 byte values are stored in the memory location 0xA2 and up.
  for (start = 1; start <= 6; start++) {
    Wire.beginTransmission(MS5611);                    //Start communication with the MPU-6050.
    Wire.write(0xA0 + start * 2);                              //Send the address that we want to read.
    Wire.endTransmission();                                    //End the transmission.

    Wire.requestFrom(MS5611, 2);                       //Request 2 bytes from the MS5611.
    C[start] = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the C[x] calibration variable.
  }

  OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
  SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

  actual_pressure = 0;                                          //Reset the pressure calculations.

  loop_timer = micros();                                        //Set the timer for the first loop.
}

//******************************************************************************************************************************************************

void loop() {

  altimeter();

    Serial.println(altitude);

  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  //Serial.println(micros() - loop_timer);
  loop_timer = micros();                                                           //Set the timer for the next loop.
}

//***************************************************************************************************************************************************
