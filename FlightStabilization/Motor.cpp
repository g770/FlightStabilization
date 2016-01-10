
#include "Motor.h"

// Min and max throttle pulse width in microseconds
// These are typical values used by ESCs
const int MIN_THROTTLE = 1000;
const int MAX_THROTTLE = 2000;

void Motor::init(uint8_t pin, int minThrottle, int maxThrottle)
{
	this->minThrottle = minThrottle;
	this->maxThrottle = maxThrottle;
	this->motorControl.attach(pin);
}

void Motor::arm()
{
	// Arm the motor by sending a throttle off command
	this->motorControl.writeMicroseconds(MIN_THROTTLE);
}

void Motor::writeThrottle(int throttleValue)
{
	if (!this->throttleValueValid(throttleValue))
	{
		return;
	}

	// First map the throttle value into the correct pulse
	// width to send to the motor, then write it
	int microsecs = map(throttleValue, this->minThrottle, this->maxThrottle, MIN_THROTTLE, MAX_THROTTLE);

	this->motorControl.writeMicroseconds(microsecs);
}

bool Motor::throttleValueValid(int throttleValue)
{
	if (throttleValue < this->minThrottle || throttleValue > this->maxThrottle)
	{
		return false;
	}

	return true;
}