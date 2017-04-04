// MotorThrottleValue.h

#ifndef _MOTORTHROTTLEVALUE_h
#define _MOTORTHROTTLEVALUE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class MotorThrottleValue
{
public:
	void setThrottleValue(uint16_t value);
	uint16_t getThrottleValue() { return this->throttleValue; }

	void increase(uint32_t increment);
	void decrease(uint32_t increment);

private:
	uint16_t throttleValue;
};

#endif

