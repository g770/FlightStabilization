// TimeInterval.h

#ifndef _TIMEINTERVAL_h
#define _TIMEINTERVAL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class TimeInterval
{

	static TimeInterval CreateFromMicroseconds(uint32_t);
	static TimeInterval CreateFromMilliseconds(uint32_t);

 public:
	// TODO: Fix access
	 TimeInterval(uint32_t val) : value(val) { }

	 uint32_t getMicroSeconds();
	 uint32_t getMilliseconds();

private:
	TimeInterval() { }

	uint32_t value;
};



#endif

