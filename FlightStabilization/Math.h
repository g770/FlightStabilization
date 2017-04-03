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
	// Max value of the uint16_t type
	static const uint16_t UINT16_T_MAX = (uint16_t)-1;

	// Min value of the uint16_t type
	static const uint16_t UINT16_T_MIN = 0;

	// Converts radians to degrees
	static double radianToDegrees(double rad);

	// Maps a double value from the input range (min, max) to the output range
	static double map_double(double x, double in_min, double in_max, double out_min, double out_max)
	{
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	// Rounds a double to an int.  If the double is negative, returns 0
	static uint32_t roundToInt(double d)
	{
		uint32_t retVal = 0;
		if (d > 0.0) {
			retVal = (uint32_t)(d + 0.5);
		}

		return retVal;
	}

	// Add 2 values without overflowing.  If the addition would result in 
	// an overflow, the max value of the input type is returned.
	static uint16_t addWithoutOverflow(uint16_t value, uint32_t increment)
	{
		uint16_t result = value + increment;
		if (result < value) {
			result = UINT16_T_MAX;
		}

		return result;
	}

	// Subtracts 2 values without overflowing.  If the subtraction would result in
	// an overflow, the min value of the input type is returned.
	static uint16_t subtractWithoutOverflow(uint16_t value, uint32_t increment)
	{
		uint16_t result = value - increment;
		if (result > value) {
			result = UINT16_T_MIN;
		}

		return result;
	}

};

#endif

