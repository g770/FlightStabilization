#ifndef _DEADBAND_h
#define _DEADBAND_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// Stores parameters for the deadband on a channel
// When pulse widths within the range are observed, the 
// deadband value is used
struct Deadband
{
	uint32_t lowEnd;
	uint32_t highEnd;
	uint32_t deadbandValue;
};

#endif

