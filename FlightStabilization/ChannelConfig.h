// ChannelConfig.h

#ifndef _CHANNELCONFIG_h
#define _CHANNELCONFIG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "RCRadio.h"

class ChannelConfig
{

public:
	static long getChannelMin(RCRadio::Channel channel);
	static long getChannelMax(RCRadio::Channel channel);
};

#endif

