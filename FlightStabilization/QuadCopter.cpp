#include "CommonDefs.h"
#include "QuadCopter.h"
#include "PinConfiguration.h"
#include "ChannelConfig.h"
#include "Math.h"

// Constants defining the motor positions
const int TOP_LEFT_MOTOR = 0;
const int TOP_RIGHT_MOTOR = 1;
const int BOTTOM_LEFT_MOTOR = 2;
const int BOTTOM_RIGHT_MOTOR = 3;

void QuadCopter::init()
{
	if (!this->imu.begin())
	{
		DEBUG_PRINTLN("Failed to initialize IMU");
		while (1);
	}
	else
	{
		DEBUG_PRINTLN("IMU Initialized");
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

	// Arm the motors
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		this->motors[i].arm();
		DEBUG_PRINT("Armed motor: ");
		DEBUG_PRINTLN(i);
	}

	DEBUG_PRINTLN("Quadcopter: Initialized");
}


void QuadCopter::update()
{
	// Core Control loop

	// TODO: Bit of a hack, calling directly into the pwmreader from here
	PWMReader::update();

	// Read current motor values
	uint16_t newTopLeftMotor = this->motors[TOP_LEFT_MOTOR].getCurrentThrottle();
	uint16_t newBottomLeftMotor = this->motors[BOTTOM_LEFT_MOTOR].getCurrentThrottle();
	uint16_t newTopRightMotor = this->motors[TOP_RIGHT_MOTOR].getCurrentThrottle();
	uint16_t newBottomRightMotor = this->motors[BOTTOM_RIGHT_MOTOR].getCurrentThrottle();

	// Read the accelerometer
	imu::Vector<3> accelerometer = this->imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

	DEBUG_PRINT("Accel (x, y, z): ");
	DEBUG_PRINT(Math::radianToDegrees(accelerometer.x()));
	DEBUG_PRINT(", ");
	DEBUG_PRINT(Math::radianToDegrees(accelerometer.y()));
	DEBUG_PRINT(", ");
	DEBUG_PRINTLN(Math::radianToDegrees(accelerometer.z()));

	// Process each channel to calculate the new motor values
	processThottleChannel(newTopLeftMotor, newBottomLeftMotor, newTopRightMotor, newBottomRightMotor);
	processRollChannel(accelerometer, newTopLeftMotor, newBottomLeftMotor, newTopRightMotor, newBottomRightMotor);
	processPitchChannel(accelerometer, newTopLeftMotor, newBottomLeftMotor, newTopRightMotor, newBottomRightMotor);
	processYawChannel(accelerometer, newTopLeftMotor, newBottomLeftMotor, newTopRightMotor, newBottomRightMotor);

	// Write the new motor values
	this->motors[TOP_LEFT_MOTOR].writeThrottle(newTopLeftMotor);
	this->motors[BOTTOM_LEFT_MOTOR].writeThrottle(newBottomLeftMotor);
	this->motors[TOP_RIGHT_MOTOR].writeThrottle(newTopRightMotor);
	this->motors[BOTTOM_RIGHT_MOTOR].writeThrottle(newBottomRightMotor);
}

void QuadCopter::processThottleChannel(uint16_t &topLeftOut, uint16_t &bottomLeftOut, uint16_t &topRightOut, uint16_t &bottomRightOut)
{
	long throttleChannel;
	bool result = this->receiver.readChannel(RCRadio::THROTTLE, &throttleChannel);

	if (result)
	{
		// Calc correction once and apply to all
		double correction;
		double error;
		this->throttlePID.calculateCorrection(topLeftOut, throttleChannel, error, correction);

		topLeftOut += correction;
		bottomLeftOut += correction;
		topRightOut += correction;
		bottomRightOut += correction;
	}
}

void QuadCopter::processPitchChannel(imu::Vector<3> &accelerometer, uint16_t &topLeftOut, uint16_t &bottomLeftOut, uint16_t &topRightOut, uint16_t &bottomRightOut)
{
	long pitchChannel;
	bool result = this->receiver.readChannel(RCRadio::PITCH, &pitchChannel);

	if (result)
	{
		// Positive acceleromoter value is pitch forward, negative is pitch back
		double correction;
		double error;
		this->pitchPID.calculateCorrection(Math::radianToDegrees(accelerometer.y()), pitchChannel, error, correction);

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

void QuadCopter::processRollChannel(imu::Vector<3> &accelerometer, uint16_t &topLeftOut, uint16_t &bottomLeftOut, uint16_t &topRightOut, uint16_t &bottomRightOut)
{
	long rollChannel;
	bool result = this->receiver.readChannel(RCRadio::ROLL, &rollChannel);

	if (result)
	{
		// Positive accelerometer is roll right, negative is roll left
		double correction;
		double error;
		this->rollPID.calculateCorrection(Math::radianToDegrees(accelerometer.x()), rollChannel, error, correction);

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

void QuadCopter::processYawChannel(imu::Vector<3> &accelerometer, uint16_t &topLeftOut, uint16_t &bottomLeftOut, uint16_t &topRightOut, uint16_t &bottomRightOut)
{
	long rollChannel;
	bool result = this->receiver.readChannel(RCRadio::YAW, &rollChannel);

	if (result)
	{
		// + accelerometer is yaw left, - is yaw right
		double correction;
		double error;
		this->yawPID.calculateCorrection(Math::radianToDegrees(accelerometer.z()), rollChannel, error, correction);

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

