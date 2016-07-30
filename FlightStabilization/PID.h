// PID.h

#ifndef _PID_h
#define _PID_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// Represents a single PID controller
class PID
{
public:
	// Sets the proportional, integral and derivative gains
	void setPIDConstants(double p, double i, double d);

	// Core PID calculation.  Takes the actual value (of a control), the desired value
	// and returns the correction to apply calcuated by the PID algorithm.  The actual
	// error value (desired - actual) is also returned.
	void calculateCorrection(double actual, double desired, double &errorOut, double &correctionOut);

	// Sets the PID state -- clears any accumulated error
	void reset();

private:
	double proportional;
	double integral;
	double derivative;

	double accumulatedError;
	double previousActual;
};
#endif

