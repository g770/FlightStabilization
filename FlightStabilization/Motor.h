// Motor.h

#ifndef _MOTOR_h
#define _MOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Servo.h>

class Motor
{

 public:
	 // Initialize the motor class
	 // pin -- The pin the motor is connected to
	 // minThrottle, maxThrottle -- The min and max values that may be 
	 // sent to the writeThrottle command.  These are typically the values 
	 // read from a receiver.
	 void init(uint8_t pin, int minThottle, int maxThrottle);
	 
	 // Arms the motor
	 void arm();

	 // Writes a throttle value to the motor
	 void writeThrottle(int throttleValue);

 private:
	 bool throttleValueValid(int throttleValue);

	Servo motorControl;
	int minThrottle;
	int maxThrottle;
};


#endif

