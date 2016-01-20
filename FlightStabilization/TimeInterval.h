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

public:
	static TimeInterval CreateFromMicroseconds(uint32_t);
	static TimeInterval CreateFromMilliseconds(uint32_t);

	 uint32_t getMicroSeconds();
	 uint32_t getMilliseconds();

	 friend bool operator<=(TimeInterval& t1, TimeInterval& t2);
	 friend bool operator>=(TimeInterval& t1, TimeInterval& t2);

	 // TODO: Fix access
	 TimeInterval(uint32_t val) : value(val) { }
protected:

	uint32_t getValue();

 private:
	 TimeInterval() { }

	uint32_t value;
};



#endif

