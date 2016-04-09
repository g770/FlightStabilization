// 
// 
// 

#include "ChannelConfig.h"
#include "Motor.h"

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
