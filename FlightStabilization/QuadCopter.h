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
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

class QuadCopter
{

 public:
	 void init();
	 void update();

 private:
	 static const int NUM_MOTORS = 1;

	 RCRadio receiver;
	 Motor motors[NUM_MOTORS];
	 Adafruit_BNO055 imu;
};


#endif

