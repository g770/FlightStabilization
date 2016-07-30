// Math.h

#ifndef _MATH_h
#define _MATH_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// General math helper functions
class Math
{
public:
	// Converts radians to degrees
	static double radianToDegrees(double rad);

	// Maps a double value from the input range (min, max) to the output range
	static double map_double(double x, double in_min, double in_max, double out_min, double out_max)
	{
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
};

#endif

