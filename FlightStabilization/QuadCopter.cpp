#include "CommonDefs.h"
#include "QuadCopter.h"
#include "PinConfiguration.h"

// Constants defining the motor positions
const int TOP_LEFT_MOTOR = 0;
const int TOP_RIGHT_MOTOR = 1;
const int BOTTOM_LEFT_MOTOR = 2;
const int BOTTOM_RIGHT_MOTOR = 3;

void QuadCopter::init()
{
	// Setup the receiver (for now just the throttle channel)
	this->receiver.configureChannel(RCRadio::THROTTLE, PinConfiguration::THROTTLE_PIN, RCRadio::NO_SCALING, RCRadio::NO_SCALING);

	// Initialize the motors
	this->motors[TOP_LEFT_MOTOR].init(PinConfiguration::TOP_LEFT_MOTOR_PIN);
	this->motors[TOP_RIGHT_MOTOR].init(PinConfiguration::TOP_RIGHT_MOTOR_PIN);
	this->motors[BOTTOM_LEFT_MOTOR].init(PinConfiguration::BOTTOM_LEFT_MOTOR_PIN);
	this->motors[BOTTOM_RIGHT_MOTOR].init(PinConfiguration::BOTTOM_RIGHT_MOTOR_PIN);

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
