#include "CommonDefs.h"
#include "QuadCopter.h"


// Constants defining the motor positions
const int TOP_LEFT_MOTOR = 0;
const int TOP_RIGHT_MOTOR = 1;
const int BOTTOM_LEFT_MOTOR = 2;
const int BOTTOM_RIGHT_MOTOR = 3;

const int MIN_THROTTLE = 0;
const int MAX_THROTTLE = 100;

void QuadCopter::init()
{
	// Setup the receiver (for now just the throttle channel)
	// TODO: Correct the pin parameter
	// Set the throttle channel to return values between 0 and 100
	this->receiver.configureChannel(RCRadio::THROTTLE, 0, MIN_THROTTLE, MAX_THROTTLE);

	// Initialize the motors
	// TODO: Correct the motor pins
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		this->motors[i].init(0);
		this->motors[i].arm();
	}

	DEBUG_PRINTLN("Quadcopter: Initialized");
}

void QuadCopter::update()
{
	// Read the throttle channel
	long throttleChannel = this->receiver.readChannel(RCRadio::THROTTLE);

	// Write the new throttle channel value to the motors
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		DEBUG_PRINT("Quadcopter: Writing motor ");
		DEBUG_PRINTLN(i);
		this->motors[i].writeThrottle(throttleChannel);
	}

}
