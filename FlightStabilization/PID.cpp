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

	double dt = actual - this->previousActual;
	correctionOut = (this->proportional * absError) + (this->integral * this->accumulatedError) + 
		(this->derivative * dt);

	this->previousActual = actual;
}

void PID::reset()
{
	this->accumulatedError = 0.0;
	this->previousActual = 0.0;
}
