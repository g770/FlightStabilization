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
	 void init(uint8_t pin);
	 
	 // Arms the motor
	 void arm();

	 // Writes a throttle value to the motor
	 // throttleValue - A value between 0 and 100
	 void writeThrottle(uint8_t throttleValue);

 private:
	 bool throttleValueValid(uint8_t throttleValue);

	Servo motorControl;
	int minThrottle;
	int maxThrottle;
};


#endif

