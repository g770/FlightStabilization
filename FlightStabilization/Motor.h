// Motor.h

#ifndef _MOTOR_h
#define _MOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Servo.h>

class Motor
{

public:
	static const uint16_t MIN_THROTTLE_IN = 0;
	static const uint16_t MAX_THROTTLE_IN = 1000;

	// Initialize the motor class
	// pin -- The pin the motor is connected to
	void init(uint8_t pin);

	// Arms the motor
	void arm();

	// Shuts off the motor
	void off();

	// Writes a throttle value to the motor
	// throttleValue - A value between 0 and 1000
	// Returns true if write was successful, false in a failed write or invalid value
	bool writeThrottle(uint16_t throttleValue);

	uint16_t getCurrentThrottle() { return this->currentThrottle; }

 private:
	 bool throttleValueValid(uint16_t throttleValue);

	uint16_t currentThrottle;
	Servo motorControl;
};


#endif

