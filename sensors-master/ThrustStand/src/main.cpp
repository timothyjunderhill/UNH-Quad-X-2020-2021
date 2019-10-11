#include <Arduino.h>



int desiredPulse = 0;
int actualPulse = 1000; 
int counter = 0;


 // Get maximum value for selected PWM resolution (100% Duty)
 int pwmMax = 256;
 // Initializing pulse for ESCs, 25% duty
 int escInit = pwmMax/4;

 int pwmFreq = 250;
 int pwmRes = 8;
 int escPulseTime = 4000;

 // Value for analog write function 
 int escPulsePWM = 0;

 // esc pins 
int escOut1 = 6;

void EscInitialize()
{

	// Escs
	pinMode(escOut1,OUTPUT);

	// All on Timer FTM0 -> pwmFreq
	analogWriteFrequency(escOut1, pwmFreq);

	// Set PWM resolution
	analogWriteResolution(pwmRes);
	
	// Initialize ESCs
	analogWrite(escOut1, escInit);
	delay(5000);
	Serial.println("ESC Initialization Completed");	

	return;
}
void PulsetoPWM()
{
	// Convert Mircosecond time to PWM pulse for motors
	escPulsePWM = actualPulse*pwmMax/escPulseTime;

	// Send PWM pulse to motors
	analogWrite(escOut1, escPulsePWM);

	return;
}

void GetDesiredPulse()
{
	 // Read the input pulse variable from the Serial monitor 
	if(Serial.available() > 0)                                // Wait for avaible bytes 
	{
		counter++;
		char inChar = Serial.read();                            // Read the incoming bytes 
		if(inChar >= '0' && inChar <= '9')                      // check to make sure the value is a desiredPulse 
		{
			desiredPulse*=10;                                     // Multiplies the prevous value by 10 to stack the values 
			desiredPulse+= (inChar - '0');                        // Convert the ascii to number and add to value
		}
		else                                                    // Typed in some other character, restart 
		{
			Serial.println("Please input an integer");          
			desiredPulse = 0;                                     // Reset the desired Pulse 
			counter = 0;                                          // Reset counter 
		}
	}
	if( counter == 4)                                         // Once the pulse is typed in, display 
	{
		Serial.print("Desired Pulse Length: ");                 
		Serial.println(desiredPulse);                           // Show the user desirePulse 
		delay(1000);

	}
	if( counter == 4 && (desiredPulse > 2000 || desiredPulse < 1000) )
	{
		Serial.println("The desired pulse is not within the 1000-2000 bounds.");
		desiredPulse = 0;
		counter = 0;
	}
	return;
}

void ConvergePulse()
{
	if( counter == 4 && desiredPulse >= 1000 && desiredPulse <= 2000) // Check to make sure the desiredPulse is within the bounds 
	{
		Serial.print("Desired Pulse: ");
		Serial.println(desiredPulse);
		Serial.print("Actual Pulse: ");
		Serial.println(actualPulse);
		delay(1000);
		while(desiredPulse > actualPulse)
		{
			actualPulse++;
			delay(10);
			Serial.println(actualPulse);
			PulsetoPWM();    		// Convert pulse to  PWM signal 
		}
		while(desiredPulse < actualPulse)
		{
			actualPulse--;
			delay(10);
			Serial.println(actualPulse);
			PulsetoPWM();    		// Convert pulse to  PWM signal 
		}
		Serial.println("Converged");
		Serial.print("Desired Pulse: ");
		Serial.println(desiredPulse);
		Serial.print("Actual Pulse: ");
		Serial.println(actualPulse);

		desiredPulse = 0;
		counter = 0;
	}
	return;
}

void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	while(!Serial);
	EscInitialize(); // Initialize the ESCs 
	delay(500);
	Serial.println("Input a desiredPulse between 1000 and 2000");
}

void loop() {
	GetDesiredPulse();   	// Get the desired motor pulse 
	ConvergePulse();   		// Converge the actual pulse to the desired pulse
}
