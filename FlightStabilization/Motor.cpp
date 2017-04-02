
#include "Motor.h"
#include "CommonDefs.h"

// Min and max throttle pulse width in microseconds
// These are typical values used by ESCs
const int MIN_THROTTLE = 1060;
const int MAX_THROTTLE = 1860;

const int MOTOR_OFF_PULSE = 0;

// The pulse length to send when arming the motor
const int ARMING_PULSE = 1000;

// How many pulses to send when arming
const int ARMING_PULSE_COUNT = 50;

// Time delay between arming pulses
const int ARMING_PULSE_DELAY = 20;

void Motor::init(uint8_t pin)
{
	this->motorControl.attach(pin);
	this->pin = pin;

	DEBUG_PRINT("Motor: Attached to pin ");
	DEBUG_PRINTLN(pin);
}

void Motor::arm()
{
	// ESCs are armed (typically) by spending a repeated sequence of low throttle signals
	for (int i = 0; i < ARMING_PULSE_COUNT; i++)
	{
		this->motorControl.writeMicroseconds(ARMING_PULSE);
	}

	// After the motor is armed, start the motor with a min throttle
	//this->motorControl.writeMicroseconds(MIN_THROTTLE);
	this->currentThrottle = MIN_THROTTLE_IN;
}

void Motor::off()
{
	this->motorControl.writeMicroseconds(MOTOR_OFF_PULSE);
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


	//DEBUG_PRINT("Motor ");
	//DEBUG_PRINT(pin);
	//DEBUG_PRINT(", ");
	//DEBUG_PRINTLN(throttleValue);

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