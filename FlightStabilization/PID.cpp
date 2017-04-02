// 
// 
// 

#include "PID.h"
#include "CommonDefs.h"

void PID::setPIDConstants(double p, double i, double d)
{
	this->proportional = p;
	this->integral = i;
	this->derivative = d;
}

void PID::calculateCorrection(double actual, double desired, double &errorOut, double &correctionOut, bool debug) const
{
	errorOut = desired - actual;

	this->accumulatedError += errorOut;  

	double dt = actual - this->previousActual;
	correctionOut = abs((this->proportional * errorOut) + (this->integral * this->accumulatedError) + 
		(this->derivative * dt));

	this->previousActual = actual;
	if (debug)
	{
		DEBUG_PRINT("Actual, desired: ");
		DEBUG_PRINT(actual);
		DEBUG_PRINT(", ");
		DEBUG_PRINTLN(desired);
		DEBUG_PRINT("Error, correction: ");
		DEBUG_PRINT(errorOut);
		DEBUG_PRINT(", ");
		DEBUG_PRINTLN(correctionOut);
	}

}

void PID::reset()
{
	this->accumulatedError = 0.0;
	this->previousActual = 0.0;
}
