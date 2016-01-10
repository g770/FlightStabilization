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
	 void init(uint8_t pin, int minThottle, int maxThrottle);
	 void arm();
	 void writeThrottle(int throttleValue);

 private:
	Servo motorControl;
	int minThrottle;
	int maxThrottle;
};


#endif

