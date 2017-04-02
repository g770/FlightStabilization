// 
// 
// 

#include "ChannelConfig.h"
#include "Motor.h"

// Deadband parameters for each channel
static const Deadband pitchDB{ 1480, 1520, 1500 };
static const Deadband rollDB{ 1480, 1520, 1500 };
static const Deadband yawDB{ 1480, 1520, 1500 };

long ChannelConfig::getChannelMin(RCRadio::Channel channel)
{
	long retVal;

	switch (channel)
	{
	case RCRadio::Channel::PITCH:
		retVal = -45;
		break;
	case RCRadio::Channel::ROLL:
		retVal = -45;
		break;
	case RCRadio::Channel::THROTTLE:
		retVal = Motor::MIN_THROTTLE_IN;
		break;
	case RCRadio::Channel::YAW:
		retVal = -45;
		break;
	}

	return retVal;
}

long ChannelConfig::getChannelMax(RCRadio::Channel channel)
{
	long retVal;

	switch (channel)
	{
	case RCRadio::Channel::PITCH:
		retVal = 45;
		break;
	case RCRadio::Channel::ROLL:
		retVal = 45;
		break;
	case RCRadio::Channel::THROTTLE:
		retVal = Motor::MAX_THROTTLE_IN;
		break;
	case RCRadio::Channel::YAW:
		retVal = 45;
		break;
	}

	return retVal;
}

const Deadband* ChannelConfig::getChannelDeadband(RCRadio::Channel channel)
{
	const Deadband *retVal = NULL;

	switch (channel)
	{
	case RCRadio::Channel::PITCH:
		retVal = &pitchDB;
		break;
	case RCRadio::Channel::ROLL:
		retVal = &rollDB;
		break;
	case RCRadio::Channel::THROTTLE:
		retVal = NULL;
		break;
	case RCRadio::Channel::YAW:
		retVal = &yawDB;
		break;
	}

	return retVal;
}

