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
#include "PID.h"

class QuadCopter
{

 public:
	 void init();
	 void update();

 private:
	 void processThottleChannel(uint16_t&, uint16_t&, uint16_t&, uint16_t&);
	 void processPitchChannel(imu::Vector<3>&, uint16_t&, uint16_t&, uint16_t&, uint16_t&);
	 void processRollChannel(imu::Vector<3>&, uint16_t&, uint16_t&, uint16_t&, uint16_t&);
	 void processYawChannel(imu::Vector<3>&, uint16_t&, uint16_t&, uint16_t&, uint16_t&);

	 static const int NUM_MOTORS = 1;

	 RCRadio receiver;
	 Motor motors[NUM_MOTORS];
	 Adafruit_BNO055 imu;
	 PID throttlePID;
	 PID rollPID;
	 PID pitchPID;
	 PID yawPID;
};


#endif

