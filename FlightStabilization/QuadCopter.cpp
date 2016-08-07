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
	this->throttlePID.setPIDConstants(.01, 0, 0);
	this->rollPID.setPIDConstants(.01, 1, 1);
	this->pitchPID.setPIDConstants(.01, 1, 1);
	this->yawPID.setPIDConstants(.01, 1, 1);

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


void QuadCopter::update()
{
	// Core Control loop

	// TODO: Bit of a hack, calling directly into the pwmreader from here to process interrupts
	PWMReader::update();

	// Read the receiver
	RCRadio::ChannelData channelData;
	this->receiver.readChannels(channelData);

	// In the update loop there are two states, motors armed and unarmed.  If we are in an
	// armed state the motors are running and we'll perform flight stabilization.  If we are in
	// an unarmed state we watch for the arming command.
	if (isArmed)
	{
		// Read current motor values
		uint16_t newTopLeftMotor = this->motors[TOP_LEFT_MOTOR].getCurrentThrottle();
		uint16_t newBottomLeftMotor = this->motors[BOTTOM_LEFT_MOTOR].getCurrentThrottle();
		uint16_t newTopRightMotor = this->motors[TOP_RIGHT_MOTOR].getCurrentThrottle();
		uint16_t newBottomRightMotor = this->motors[BOTTOM_RIGHT_MOTOR].getCurrentThrottle();

		// Read the accelerometer
		imu::Vector<3> accelerometer = this->imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

		//DEBUG_PRINT("Accel (x, y, z): ");
		//DEBUG_PRINT(Math::radianToDegrees(accelerometer.x()));
		//DEBUG_PRINT(", ");
		//DEBUG_PRINT(Math::radianToDegrees(accelerometer.y()));
		//DEBUG_PRINT(", ");
		//DEBUG_PRINTLN(Math::radianToDegrees(accelerometer.z()));

		// Process each channel to calculate the new motor values
		processThottleChannel(channelData, newTopLeftMotor, newBottomLeftMotor, newTopRightMotor, newBottomRightMotor); 
		//processRollChannel(channelData, accelerometer, newTopLeftMotor, newBottomLeftMotor, newTopRightMotor, newBottomRightMotor);
		//processPitchChannel(channelData, accelerometer, newTopLeftMotor, newBottomLeftMotor, newTopRightMotor, newBottomRightMotor);
		//processYawChannel(channelData, accelerometer, newTopLeftMotor, newBottomLeftMotor, newTopRightMotor, newBottomRightMotor);

		// Write the new motor values
		this->motors[TOP_LEFT_MOTOR].writeThrottle(newTopLeftMotor);
		this->motors[BOTTOM_LEFT_MOTOR].writeThrottle(newBottomLeftMotor);
		this->motors[TOP_RIGHT_MOTOR].writeThrottle(newTopRightMotor);
		this->motors[BOTTOM_RIGHT_MOTOR].writeThrottle(newBottomRightMotor);

		// Check if the disarming command has been given
		processDisarmingCommand(channelData);
	}
	else
	{
		//processArmingCommand(channelData);
		// Debug
		this->armMotors();
		this->isArmed = true;
	}
}


bool QuadCopter::isInArmingCommandErrorBounds(long channelData, long channelEndpoint)
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

void QuadCopter::processThottleChannel(RCRadio::ChannelData &channelData, uint16_t &topLeftOut, uint16_t &bottomLeftOut, uint16_t &topRightOut, uint16_t &bottomRightOut)
{
	if (channelData.channelResults[RCRadio::THROTTLE])
	{
		// Calc correction once and apply to all
		double correction;
		double error;
		this->throttlePID.calculateCorrection(topLeftOut, channelData.channelData[RCRadio::THROTTLE], error, correction);

		topLeftOut += correction;
		bottomLeftOut += correction;
		topRightOut += correction;
		bottomRightOut += correction;

		DEBUG_PRINT("Throttle: ");
		DEBUG_PRINT(topLeftOut);
		DEBUG_PRINT(", ");
		DEBUG_PRINT(topRightOut);
		DEBUG_PRINT(", ");
		DEBUG_PRINT(bottomLeftOut);
		DEBUG_PRINT(", ");
		DEBUG_PRINTLN(bottomRightOut);

	}
}

void QuadCopter::processPitchChannel(RCRadio::ChannelData &channelData, imu::Vector<3> &accelerometer, uint16_t &topLeftOut, uint16_t &bottomLeftOut, uint16_t &topRightOut, uint16_t &bottomRightOut)
{
	if (channelData.channelResults[RCRadio::PITCH])
	{
		// Positive acceleromoter value is pitch forward, negative is pitch back
		double correction;
		double error;
		this->pitchPID.calculateCorrection(Math::radianToDegrees(accelerometer.y()), channelData.channelData[RCRadio::PITCH], error, correction);

		// If error is positive, accelerate forward by speeding up the rear motors and slowing the fronts
		if (error > 0)
		{
			topLeftOut -= correction;
			bottomLeftOut += correction;
			topRightOut -= correction;
			bottomRightOut += correction;
		}
		else
		{
			topLeftOut += correction;
			bottomLeftOut -= correction;
			topRightOut += correction;
			bottomRightOut -= correction;
		}
	}
}

void QuadCopter::processRollChannel(RCRadio::ChannelData &channelData, imu::Vector<3> &accelerometer, uint16_t &topLeftOut, uint16_t &bottomLeftOut, uint16_t &topRightOut, uint16_t &bottomRightOut)
{

	if (channelData.channelResults[RCRadio::ROLL])
	{
		// Positive accelerometer is roll right, negative is roll left
		double correction;
		double error;
		this->rollPID.calculateCorrection(Math::radianToDegrees(accelerometer.x()), channelData.channelData[RCRadio::ROLL], error, correction);

		// If error is positive, roll right by speeding up the left motors and slowing down the right
		if (error > 0)
		{
			topLeftOut += correction;
			bottomLeftOut += correction;
			topRightOut -= correction;
			bottomRightOut -= correction;
		}
		else
		{
			topLeftOut -= correction;
			bottomLeftOut -= correction;
			topRightOut += correction;
			bottomRightOut += correction;
		}
	}
}

void QuadCopter::processYawChannel(RCRadio::ChannelData &channelData, imu::Vector<3> &accelerometer, uint16_t &topLeftOut, uint16_t &bottomLeftOut, uint16_t &topRightOut, uint16_t &bottomRightOut)
{
	if (channelData.channelResults[RCRadio::YAW])
	{
		// + accelerometer is yaw left, - is yaw right
		double correction;
		double error;
		this->yawPID.calculateCorrection(Math::radianToDegrees(accelerometer.z()), channelData.channelData[RCRadio::YAW], error, correction);

		// If error is positive, yaw right
		if (error > 0)
		{
			topLeftOut += correction;
			bottomLeftOut -= correction;
			topRightOut -= correction;
			bottomRightOut += correction;
		}
		else
		{
			topLeftOut -= correction;
			bottomLeftOut += correction;
			topRightOut += correction;
			bottomRightOut -= correction;
		}
	}
}

