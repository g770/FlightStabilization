
#include "Motor.h"
#include "CommonDefs.h"

// Min and max throttle pulse width in microseconds
// These are typical values used by ESCs
const int MIN_THROTTLE = 1060;
const int MAX_THROTTLE = 1860;

const int ARMING_PULSE = 1000;
const int ARMING_PULSE_COUNT = 100;
const int ARMING_PULSE_DELAY = 19;

void Motor::init(uint8_t pin)
{
	this->motorControl.attach(pin);

	DEBUG_PRINT("Motor: Attached to pin ");
	DEBUG_PRINTLN(pin);
}

void Motor::arm()
{
	// ESCs are armed (typically) by spending a repeated sequence of low throttle signals
	for (int i = 0; i < ARMING_PULSE_COUNT; i++)
	{
		this->motorControl.writeMicroseconds(ARMING_PULSE);
		delay(ARMING_PULSE_DELAY);
	}

	delay(5000);

	// After the motor is armed, start the motor with a min throttle
	this->motorControl.writeMicroseconds(MIN_THROTTLE);

}

void Motor::off()
{
	this->motorControl.writeMicroseconds(0);
}

bool Motor::writeThrottle(uint16_t throttleValue)
{
	if (!this->throttleValueValid(throttleValue))
	{
		return false;
	}

	// First map the throttle value into the correct pulse
	// width to send to the motor, then write it
	int microsecs = map(throttleValue, MIN_THROTTLE_IN, MAX_THROTTLE_IN, MIN_THROTTLE, MAX_THROTTLE);


	//DEBUG_PRINT("Motor: Write throttle value ");
	//DEBUG_PRINTLN(microsecs);

	this->motorControl.writeMicroseconds(microsecs);
	this->currentThrottle = throttleValue;

	return true;
}

inline bool Motor::throttleValueValid(uint16_t throttleValue)
{
	if (throttleValue < MIN_THROTTLE_IN || throttleValue > MAX_THROTTLE_IN)
	{
		return false;
	}

	return true;
}