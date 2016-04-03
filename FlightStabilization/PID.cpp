// 
// 
// 

#include "PID.h"

void PID::setPIDConstants(double p, double i, double d)
{
	this->proportional = p;
	this->integral = i;
	this->derivative = d;
}

void PID::calculateCorrection(double actual, double desired, double &errorOut, double &correctionOut)
{
	errorOut = desired - actual;
	
	double absError = abs(errorOut);
	this->accumulatedError += absError;  

	// Technically just a PI controller for now since the derivative isn't used
	correctionOut = (this->proportional * absError) + (this->integral * this->accumulatedError);
}

void PID::reset()
{
	this->accumulatedError = 0.0;
}
