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
	// Setup the receiver
	this->receiver.configureChannel(RCRadio::THROTTLE, PinConfiguration::THROTTLE_PIN, Motor::MIN_THROTTLE_IN, Motor::MAX_THROTTLE_IN);
	//this->receiver.configureChannel(RCRadio::ROLL, PinConfiguration::ROLL_PIN, RCRadio::NO_SCALING, RCRadio::NO_SCALING);
	//this->receiver.configureChannel(RCRadio::PITCH, PinConfiguration::PITCH_PIN, RCRadio::NO_SCALING, RCRadio::NO_SCALING);

	// Initialize the motors
	this->motors[TOP_LEFT_MOTOR].init(PinConfiguration::TOP_LEFT_MOTOR_PIN);
	this->motors[TOP_LEFT_MOTOR].arm();
	//this->motors[TOP_RIGHT_MOTOR].init(PinConfiguration::TOP_RIGHT_MOTOR_PIN);
	//this->motors[BOTTOM_LEFT_MOTOR].init(PinConfiguration::BOTTOM_LEFT_MOTOR_PIN);
	//this->motors[BOTTOM_RIGHT_MOTOR].init(PinConfiguration::BOTTOM_RIGHT_MOTOR_PIN);

	DEBUG_PRINTLN("Quadcopter: Initialized");
}

void QuadCopter::update()
{
	// TODO: Bit of a hack, calling directly into the pwmreader from here
	PWMReader::update();

	// Read the throttle channel
	long throttleChannel;
	bool result = this->receiver.readChannel(RCRadio::THROTTLE, &throttleChannel);

	//long rollChannel;
	//this->receiver.readChannel(RCRadio::ROLL, &rollChannel);

	//long pitchChannel;	
	//this->receiver.readChannel(RCRadio::PITCH, &pitchChannel);

	// Write the new throttle channel value to the motors
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		uint16_t currentThrottle = this->motors[i].getCurrentThrottle();

		// Very simple control loop.  First read the throttle channel, that is the desired
		// state.  Then read the curren throttle value and calculate how to modify the current
		// throttle to move it toward the desired state (the throttle channel value)
		int step;
		if (result)
		{
			if (throttleChannel > currentThrottle) {
				step = 1;
			}
			else if (currentThrottle == 0)
			{
				step = 0;
			}
			else
			{
				step = -1;
			}
		}
		else
		{
			step = 0;
		}

		uint16_t newThrottle = currentThrottle + step;
		//DEBUG_PRINT("Quadcopter: Writing motor ");
		//DEBUG_PRINTLN(newThrottle);
		this->motors[i].writeThrottle(newThrottle);
	}

}

