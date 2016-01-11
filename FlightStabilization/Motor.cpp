
#include "Motor.h"
#include "CommonDefs.h"

// Min and max throttle pulse width in microseconds
// These are typical values used by ESCs
const int MIN_THROTTLE = 1000;
const int MAX_THROTTLE = 2000;

void Motor::init(uint8_t pin)
{
	this->minThrottle = 0;
	this->maxThrottle = 100;
	this->motorControl.attach(pin);

	DEBUG_PRINT("Motor: Attached to pin ");
	DEBUG_PRINTLN(pin);
}

void Motor::arm()
{
	// Arm the motor by sending a throttle off command
	this->motorControl.writeMicroseconds(MIN_THROTTLE);
}

void Motor::writeThrottle(uint8_t throttleValue)
{
	if (!this->throttleValueValid(throttleValue))
	{
		return;
	}

	// First map the throttle value into the correct pulse
	// width to send to the motor, then write it
	int microsecs = map(throttleValue, this->minThrottle, this->maxThrottle, MIN_THROTTLE, MAX_THROTTLE);

	this->motorControl.writeMicroseconds(microsecs);

	DEBUG_PRINT("Motor: Write throttle value ");
	DEBUG_PRINTLN(microsecs);
}

bool Motor::throttleValueValid(uint8_t throttleValue)
{
	if (throttleValue < this->minThrottle || throttleValue > this->maxThrottle)
	{
		return false;
	}

	return true;
}