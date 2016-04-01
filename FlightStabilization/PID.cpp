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

double PID::calculateCorrection(double actual, double desired, double &errorOut, double &correctionOut)
{
	errorOut = desired - actual;
	
	double absError = abs(errorOut);
	this->accumulatedError += absError;  

	correctionOut = (this->proportional * absError) + (this->integral * this->accumulatedError);

}
