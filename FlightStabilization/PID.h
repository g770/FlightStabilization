// PID.h

#ifndef _PID_h
#define _PID_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class PID
{
public:
	void setPIDConstants(double p, double i, double d);

	double calculateCorrection(double actual, double desired, double &errorOut, double &correctionOut);

private:
	double proportional;
	double integral;
	double derivative;
};
#endif

