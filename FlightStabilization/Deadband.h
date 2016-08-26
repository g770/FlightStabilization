// Deadband.h

#ifndef _DEADBAND_h
#define _DEADBAND_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

struct Deadband
{
	uint32_t lowEnd;
	uint32_t highEnd;
	uint32_t deadbandValue;
};

#endif

