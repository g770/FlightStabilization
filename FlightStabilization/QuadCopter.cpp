#include "CommonDefs.h"
#include "QuadCopter.h"
#include "PinConfiguration.h"
#include "ChannelConfig.h"
#include "Math.h"

// Constants defining the motor positions, used as array indexes
const int TOP_LEFT_MOTOR = 0;
const int TOP_RIGHT_MOTOR = 1;
const int BOTTOM_LEFT_MOTOR = 2;
const int BOTTOM_RIGHT_MOTOR = 3;

const double ARMING_COMMAND_ERROR = 0.2;

// How long to wait after sending arming pulses to ESCs.
const int POST_ARMING_DELAY = 5000;

void QuadCopter::init()
{
	DEBUG_PRINTLN("Quadcopter: Starting init");

	if (!this->imu.begin())
	{
		DEBUG_PRINTLN("Quadcopter: Failed to initialize IMU");
		while (1);
	}
	else
	{
		DEBUG_PRINTLN("Quadcopter: IMU Initialized");
	}

	// Setup PID controllers
	this->throttlePID.setPIDConstants(.05, 0, 0);
	this->rollPID.setPIDConstants(.05, 0, 0);
	this->pitchPID.setPIDConstants(.05, .01, 0);
	this->yawPID.setPIDConstants(.05, .01, 0);

	// Setup the receiver
	this->receiver.configureChannel(RCRadio::THROTTLE, PinConfiguration::THROTTLE_PIN, 
		ChannelConfig::getChannelMin(RCRadio::THROTTLE), ChannelConfig::getChannelMax(RCRadio::THROTTLE));
	this->receiver.configureChannel(RCRadio::ROLL, PinConfiguration::ROLL_PIN, 
		ChannelConfig::getChannelMin(RCRadio::ROLL), ChannelConfig::getChannelMax(RCRadio::ROLL));
	this->receiver.configureChannel(RCRadio::PITCH, PinConfiguration::PITCH_PIN,
		ChannelConfig::getChannelMin(RCRadio::PITCH), ChannelConfig::getChannelMax(RCRadio::PITCH));
	this->receiver.configureChannel(RCRadio::YAW, PinConfiguration::YAW_PIN,
		ChannelConfig::getChannelMin(RCRadio::YAW), ChannelConfig::getChannelMax(RCRadio::YAW));

	// Initialize the motors
	this->motors[TOP_LEFT_MOTOR].init(PinConfiguration::TOP_LEFT_MOTOR_PIN);
	this->motors[TOP_RIGHT_MOTOR].init(PinConfiguration::TOP_RIGHT_MOTOR_PIN);
	this->motors[BOTTOM_LEFT_MOTOR].init(PinConfiguration::BOTTOM_LEFT_MOTOR_PIN);
	this->motors[BOTTOM_RIGHT_MOTOR].init(PinConfiguration::BOTTOM_RIGHT_MOTOR_PIN);


	DEBUG_PRINTLN("Quadcopter: Initialized");
}

int counter = 0;
void QuadCopter::update()
{
	// Core Control loop
	// TODO: Bit of a hack, calling directly into the pwmreader from here to process interrupts
	PWMReader::update();

	// Read the receiver
	RCRadio::ChannelData channelData;
	this->receiver.readChannels(channelData);

	double lastX, lastY, lastZ;

	// In the update loop there are two states, motors armed and unarmed.  If we are in an
	// armed state the motors are running and we'll perform flight stabilization.  If we are in
	// an unarmed state we watch for the arming command.
	if (isArmed)
	{
		// Read current motor values
		topLeftMotorValue.setThrottleValue(this->motors[TOP_LEFT_MOTOR].getCurrentThrottle());
		bottomLeftMotorValue.setThrottleValue(this->motors[BOTTOM_LEFT_MOTOR].getCurrentThrottle());
		topRightMotorValue.setThrottleValue(this->motors[TOP_RIGHT_MOTOR].getCurrentThrottle());
		bottomRightMotorValue.setThrottleValue(this->motors[BOTTOM_RIGHT_MOTOR].getCurrentThrottle());


		// Read the accelerometer
		imu::Vector<3> accelerometer = this->imu.getVector();

		// IMU notes:
		// -Y -> pitch forward
		// +Y -> pitch backward
		// -X -> roll left
		// +X -> roll right
		// -Z -> yaw right
		// +Z -> yaw left
		/*
		if (abs(Math::radianToDegrees(accelerometer.x()) - lastX) > 5) {
			DEBUG_PRINT("X change: ");
			DEBUG_PRINTLN(Math::radianToDegrees(accelerometer.x()));
		}

		if (abs(Math::radianToDegrees(accelerometer.y()) - lastY) > 5) {
			DEBUG_PRINT("Y change: ");
			DEBUG_PRINTLN(Math::radianToDegrees(accelerometer.y()));
		}

		if (abs(Math::radianToDegrees(accelerometer.z()) - lastZ) > 5) {
			DEBUG_PRINT("Z change: ");
			DEBUG_PRINTLN(Math::radianToDegrees(accelerometer.z()));
		}
		*/
		lastX = Math::radianToDegrees(accelerometer.x());
		lastY = Math::radianToDegrees(accelerometer.y());
		lastZ = Math::radianToDegrees(accelerometer.z());

		//DEBUG_PRINT("Accel (x, y, z): ");
		//DEBUG_PRINT(Math::radianToDegrees(accelerometer.x()));
		//DEBUG_PRINT(", ");
		//DEBUG_PRINT(Math::radianToDegrees(accelerometer.y()));
		//DEBUG_PRINT(", ");
		//DEBUG_PRINTLN(Math::radianToDegrees(accelerometer.z()));

		// Process each channel to calculate the new motor values
		processThottleChannel(channelData); 
		//processRollChannel(channelData, accelerometer);
		//processPitchChannel(channelData, accelerometer);
		processYawChannel(channelData, accelerometer);

		if ((counter % 100) == 0)
		{
		DEBUG_PRINT("TL: ");
		DEBUG_PRINT(topLeftMotorValue.getThrottleValue());
		DEBUG_PRINT(" ");
		DEBUG_PRINT("TR: ");
		DEBUG_PRINT(topRightMotorValue.getThrottleValue());
		DEBUG_PRINT(" ");
		DEBUG_PRINT("BL: ");
		DEBUG_PRINT(bottomLeftMotorValue.getThrottleValue());
		DEBUG_PRINT(" ");
		DEBUG_PRINT("BR: ");
		DEBUG_PRINTLN(bottomRightMotorValue.getThrottleValue());
		}
		counter++;

		// Write the new motor values
		this->motors[TOP_LEFT_MOTOR].writeThrottle(topLeftMotorValue.getThrottleValue());
		this->motors[BOTTOM_LEFT_MOTOR].writeThrottle(bottomLeftMotorValue.getThrottleValue());
		this->motors[TOP_RIGHT_MOTOR].writeThrottle(topRightMotorValue.getThrottleValue());
		this->motors[BOTTOM_RIGHT_MOTOR].writeThrottle(bottomRightMotorValue.getThrottleValue());

		// Check if the disarming command has been given
		//processDisarmingCommand(channelData);
	}
	else
	{
		//processArmingCommand(channelData);
		// Debug
		this->armMotors();
		this->isArmed = true;
	}
}


bool QuadCopter::isInArmingCommandErrorBounds(long channelData, long channelEndpoint) const
{
	double error = abs(channelEndpoint * ARMING_COMMAND_ERROR);

	if (abs(channelData - channelEndpoint) <= error)
	{
		return true;
	}

	return false;
}

void QuadCopter::processArmingCommand(RCRadio::ChannelData &channelData)
{
	if (channelData.channelResults[(int)RCRadio::THROTTLE])
	{
		if (this->isInArmingCommandErrorBounds(channelData.channelData[RCRadio::THROTTLE], ChannelConfig::getChannelMin(RCRadio::THROTTLE)))
		{
			throttleTotal++;
		}
		else
		{
			throttleTotal = 0;
		}
	}

	if (channelData.channelResults[(int)RCRadio::YAW])
	{
		if (this->isInArmingCommandErrorBounds(channelData.channelData[RCRadio::YAW], ChannelConfig::getChannelMax(RCRadio::YAW)))
		{
			yawTotal++;
		}
		else
		{
			yawTotal = 0;
		}
	}

	if (channelData.channelResults[(int)RCRadio::ROLL])
	{
		if (this->isInArmingCommandErrorBounds(channelData.channelData[RCRadio::ROLL], ChannelConfig::getChannelMax(RCRadio::ROLL)))
		{
			rollTotal++;
		}
		else
		{
			rollTotal = 0;
		}
	}

	if (channelData.channelResults[(int)RCRadio::PITCH])
	{
		if (this->isInArmingCommandErrorBounds(channelData.channelData[RCRadio::PITCH], ChannelConfig::getChannelMin(RCRadio::PITCH)))
		{
			pitchTotal++;
		}
		else
		{
			pitchTotal = 0;
		}
	}

	if (throttleTotal >= ARMING_COUNT && yawTotal >= ARMING_COUNT && rollTotal >= ARMING_COUNT && pitchTotal >= ARMING_COUNT)
	{
		DEBUG_PRINTLN("ARMING SIGNAL RECEIVED");

		// Arm the motors
		this->armMotors();

		this->isArmed = true;
		throttleTotal = 0;
		pitchTotal = 0;
		rollTotal = 0;
		yawTotal = 0;
	}
}

void QuadCopter::armMotors()
{
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		this->motors[i].arm();
		DEBUG_PRINT("Armed motor: ");
		DEBUG_PRINTLN(i);
	}

	delay(POST_ARMING_DELAY);
}

void QuadCopter::processDisarmingCommand(RCRadio::ChannelData &channelData)
{

	if (channelData.channelResults[(int)RCRadio::THROTTLE])
	{
		if (this->isInArmingCommandErrorBounds(channelData.channelData[RCRadio::THROTTLE], ChannelConfig::getChannelMin(RCRadio::THROTTLE)))
		{
			throttleTotal++;
		}
		else
		{
			throttleTotal = 0;
		}
	}

	if (channelData.channelResults[(int)RCRadio::YAW])
	{
		if (this->isInArmingCommandErrorBounds(channelData.channelData[RCRadio::YAW], ChannelConfig::getChannelMin(RCRadio::YAW)))
		{
			yawTotal++;
		}
		else
		{
			yawTotal = 0;
		}
	}

	if (channelData.channelResults[(int)RCRadio::ROLL])
	{
		if (this->isInArmingCommandErrorBounds(channelData.channelData[RCRadio::ROLL], ChannelConfig::getChannelMin(RCRadio::ROLL)))
		{
			rollTotal++;
		}
		else
		{
			rollTotal = 0;
		}
	}

	if (channelData.channelResults[(int)RCRadio::PITCH])
	{
		if (this->isInArmingCommandErrorBounds(channelData.channelData[RCRadio::PITCH], ChannelConfig::getChannelMin(RCRadio::PITCH)))
		{
			pitchTotal++;
		}
		else
		{
			pitchTotal = 0;
		}
	}

	if (throttleTotal >= ARMING_COUNT && yawTotal >= ARMING_COUNT && rollTotal >= ARMING_COUNT && pitchTotal >= ARMING_COUNT)
	{
		DEBUG_PRINTLN("DISARMING SIGNAL RECEIVED");

		// Disarm the motors
		this->disarmMotors();

		this->isArmed = false;
		throttleTotal = 0;
		pitchTotal = 0;
		rollTotal = 0;
		yawTotal = 0;

		// Reset the PID controllers since we have likely landed
		this->throttlePID.reset();
		this->pitchPID.reset();
		this->rollPID.reset();
		this->yawPID.reset();
	}
}

void QuadCopter::disarmMotors()
{
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		this->motors[i].off();
		DEBUG_PRINT("Shutdown motor: ");
		DEBUG_PRINTLN(i);
	}

}

void QuadCopter::processThottleChannel(RCRadio::ChannelData &channelData)
{
	if (channelData.channelResults[RCRadio::THROTTLE])
	{
		// Calc correction once and apply to all
		double correction;
		double error;
		this->throttlePID.calculateCorrection(topLeftMotorValue.getThrottleValue(), channelData.channelData[RCRadio::THROTTLE], error, correction, false);

		uint32_t roundedCorrection = Math::roundToInt(correction);

		if (error < 0)
		{
			topLeftMotorValue.decrease(roundedCorrection);
			bottomLeftMotorValue.decrease(roundedCorrection);
			topRightMotorValue.decrease(roundedCorrection);
			bottomRightMotorValue.decrease(roundedCorrection);
		}
		else
		{
			topLeftMotorValue.increase(roundedCorrection);
			bottomLeftMotorValue.increase(roundedCorrection);
			topRightMotorValue.increase(roundedCorrection);
			bottomRightMotorValue.increase(roundedCorrection);
		}
	}
}

void QuadCopter::processPitchChannel(RCRadio::ChannelData &channelData, imu::Vector<3> &accelerometer)
{
	if (channelData.channelResults[RCRadio::PITCH])
	{
		double correction;
		double error;
		this->pitchPID.calculateCorrection(Math::radianToDegrees(accelerometer.y()), channelData.channelData[RCRadio::PITCH], error, correction, false);

		uint32_t roundedCorrection = Math::roundToInt(correction);

		// If error is positive we are pitching backward, correct by speeding up the rear motors and slowing the fronts
		if (error > 0)
		{
			topLeftMotorValue.decrease(roundedCorrection);
			topRightMotorValue.decrease(roundedCorrection);

			bottomLeftMotorValue.increase(roundedCorrection);
			bottomRightMotorValue.increase(roundedCorrection);
		}
		else // pitching forward
		{
			bottomLeftMotorValue.decrease(roundedCorrection);
			bottomRightMotorValue.decrease(roundedCorrection);

			topLeftMotorValue.increase(roundedCorrection);
			topRightMotorValue.increase(roundedCorrection);

		}
	}
}

void QuadCopter::processRollChannel(RCRadio::ChannelData &channelData, imu::Vector<3> &accelerometer)
{
	if (channelData.channelResults[RCRadio::ROLL])
	{
		double correction;
		double error;
		this->rollPID.calculateCorrection(Math::radianToDegrees(accelerometer.x()), channelData.channelData[RCRadio::ROLL], error, correction, false);

		uint32_t roundedCorrection = Math::roundToInt(correction);

		// If error is positive we want to roll to the right, correct by speeding up the left motors and slowing down the right
		if (error > 0)
		{
			topLeftMotorValue.increase(roundedCorrection);
			bottomLeftMotorValue.increase(roundedCorrection);
			topRightMotorValue.decrease(roundedCorrection);
			bottomRightMotorValue.decrease(roundedCorrection);
		}
		else // Want to roll to the left, speed up the right motors and slow the left
		{
			topLeftMotorValue.decrease(roundedCorrection);
			bottomLeftMotorValue.decrease(roundedCorrection);
			topRightMotorValue.increase(roundedCorrection);
			bottomRightMotorValue.increase(roundedCorrection);
		}		
	}
}

void QuadCopter::processYawChannel(RCRadio::ChannelData &channelData, imu::Vector<3> &accelerometer)
{
	if (channelData.channelResults[RCRadio::YAW])
	{
		double correction;
		double error;
		this->yawPID.calculateCorrection(Math::radianToDegrees(accelerometer.z()), channelData.channelData[RCRadio::YAW], error, correction, false);

		uint32_t roundedCorrection = Math::roundToInt(correction);

		// If error is positive, we are yawing to the left, correct by speeding up the top right and bottom left motors
		if (error > 0)
		{
			topLeftMotorValue.decrease(roundedCorrection);
			bottomLeftMotorValue.increase(roundedCorrection);
			topRightMotorValue.increase(roundedCorrection);
			bottomRightMotorValue.decrease(roundedCorrection);
		}
		else
		{
			topLeftMotorValue.increase(roundedCorrection);
			bottomLeftMotorValue.decrease(roundedCorrection);
			topRightMotorValue.decrease(roundedCorrection);
			bottomRightMotorValue.increase(roundedCorrection);
		}
	}
}

