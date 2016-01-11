// QuadCopter.h

#ifndef _QUADCOPTER_h
#define _QUADCOPTER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "RCRadio.h"
#include "Motor.h"

class QuadCopter
{

 public:
	 void init();
	 void update();

 private:
	 static const int NUM_MOTORS = 4;

	 RCRadio receiver;
	 Motor motors[NUM_MOTORS];
};


#endif

