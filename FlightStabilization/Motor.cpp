
#include "Motor.h"

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
	this->motorControl.writeMicroseconds(MIN_THROTTLE);
}

void Motor::writeThrottle(int throttleValue)
{
	int microsecs = map(throttleValue, this->minThrottle, this->maxThrottle, MIN_THROTTLE, MAX_THROTTLE);
	this->motorControl.writeMicroseconds(microsecs);
}
