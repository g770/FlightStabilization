#include "CommonDefs.h"
#include "QuadCopter.h"
#include "PinConfiguration.h"
#include "ChannelConfig.h"

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
	this->motors[TOP_LEFT_MOTOR].arm();
	//this->motors[TOP_RIGHT_MOTOR].init(PinConfiguration::TOP_RIGHT_MOTOR_PIN);
	//this->motors[BOTTOM_LEFT_MOTOR].init(PinConfiguration::BOTTOM_LEFT_MOTOR_PIN);
	//this->motors[BOTTOM_RIGHT_MOTOR].init(PinConfiguration::BOTTOM_RIGHT_MOTOR_PIN);

	DEBUG_PRINTLN("Quadcopter: Initialized");
}


static double radianToDegrees(double rad)
{
	return rad * 57.295779513;
}

const double THROTTLE_P = 1;
const double ROLL_P = 1;
const double PITCH_P = 1;
const double YAW_P = 1;

void QuadCopter::update()
{
	// TODO: Bit of a hack, calling directly into the pwmreader from here
	PWMReader::update();

	// Read the accelerometer
	imu::Vector<3> accelerometer = this->imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

	DEBUG_PRINT("Accel (x, y, z): ");
	DEBUG_PRINT(radianToDegrees(accelerometer.x()));
	DEBUG_PRINT(", ");
	DEBUG_PRINT(radianToDegrees(accelerometer.y()));
	DEBUG_PRINT(", ");
	DEBUG_PRINTLN(radianToDegrees(accelerometer.z()));


	// Read the throttle channel
	long throttleChannel;
	bool result = this->receiver.readChannel(RCRadio::THROTTLE, &throttleChannel);

	long rollChannel;
	this->receiver.readChannel(RCRadio::ROLL, &rollChannel);

	long pitchChannel;	
	this->receiver.readChannel(RCRadio::PITCH, &pitchChannel);

	long yawChannel;
	this->receiver.readChannel(RCRadio::YAW, &yawChannel);

	uint16_t newTopLeftMotor = this->motors[TOP_LEFT_MOTOR].getCurrentThrottle();
	uint16_t newBottomLeftMotor = this->motors[BOTTOM_LEFT_MOTOR].getCurrentThrottle();
	uint16_t newTopRightMotor = this->motors[TOP_RIGHT_MOTOR].getCurrentThrottle();
	uint16_t newBottomRightMotor = this->motors[BOTTOM_RIGHT_MOTOR].getCurrentThrottle();

	// TODO: Getting thoughts in code, need to make sure signs are aligned the right way
	// Assume: positive value is right roll
	double rollError = rollChannel - radianToDegrees(accelerometer.x());
	double correction = abs(rollError) * ROLL_P;

	// If error is positive, accelerate to the right, otherwise left
	if (rollError > 0)
	{
		newTopLeftMotor += correction;
		newBottomLeftMotor += correction;
		newTopRightMotor -= correction;
		newBottomRightMotor -= correction;
	}
	else
	{
		newTopLeftMotor -= correction;
		newBottomLeftMotor -= correction;
		newTopRightMotor += correction;
		newBottomRightMotor += correction;
	}

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

