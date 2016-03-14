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

	 uint32_t getMicroSeconds() { return this->value; }

	 uint32_t getMilliseconds() { return this->value / 1000; }

	 friend bool operator<=(TimeInterval& t1, TimeInterval& t2) { return t1.getValue() <= t2.getValue(); }
	 friend bool operator>=(TimeInterval& t1, TimeInterval& t2) { return t1.getValue() >= t2.getValue(); }

	 TimeInterval() { }
	 TimeInterval(uint32_t val) : value(val) { }
protected:

	uint32_t getValue() { return this->value; }

 private:

	uint32_t value;
};



#endif

