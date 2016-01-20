
#include "Motor.h"
#include "CommonDefs.h"

// Min and max throttle pulse width in microseconds
// These are typical values used by ESCs
const int MIN_THROTTLE = 1060;
const int MAX_THROTTLE = 1860;

// Min and max input values for the throttle
//const int MIN_THROTTLE_IN = 0;
//const int MAX_THROTTLE_IN = MAX_THROTTLE - MIN_THROTTLE;
const int MIN_THROTTLE_IN = 1000;
const int MAX_THROTTLE_IN = 2000;

void Motor::init(uint8_t pin)
{
	this->motorControl.attach(pin);

	DEBUG_PRINT("Motor: Attached to pin ");
	DEBUG_PRINTLN(pin);
}

void Motor::arm()
{
	// Arm the motor by sending a throttle off command every 20 msecs
	for (int i = 0; i < 100; i++)
	{
		this->motorControl.writeMicroseconds(1000);
		delay(19);
	}

	delay(5000);
	DEBUG_PRINTLN("Motor: Armed ");

	this->motorControl.writeMicroseconds(1060);

}

// TODO: Fix to u_int_8 and have range be 0-100%
void Motor::writeThrottle(uint16_t throttleValue)
{
	if (!this->throttleValueValid(throttleValue))
	{
		return;
	}

	// First map the throttle value into the correct pulse
	// width to send to the motor, then write it
	//int microsecs = map(throttleValue, MIN_THROTTLE_IN, MAX_THROTTLE_IN, MIN_THROTTLE, MAX_THROTTLE);
	int microsecs = map(throttleValue, 1000, 2000, MIN_THROTTLE, MAX_THROTTLE);


	DEBUG_PRINT("Motor: Write throttle value ");
	DEBUG_PRINTLN(microsecs);

	this->motorControl.writeMicroseconds(microsecs);
	delay(19);
}

bool Motor::throttleValueValid(uint16_t throttleValue)
{
	if (throttleValue < MIN_THROTTLE_IN || throttleValue > MAX_THROTTLE_IN)
	{
		return false;
	}

	return true;
}