// IMUFilter.h

#ifndef _IMUFILTER_h
#define _IMUFILTER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

class IMUFilter
{
public:
	imu::Vector<3> getVector();
	bool begin() { return this->imu.begin(); }

private:
	Adafruit_BNO055 imu;
};

#endif

