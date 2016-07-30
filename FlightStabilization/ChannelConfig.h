// ChannelConfig.h

#ifndef _CHANNELCONFIG_h
#define _CHANNELCONFIG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "RCRadio.h"

// Stores configuration for a single RC radio channel
class ChannelConfig
{

public:
	// Gets the minimum pulse width a channel can return
	static long getChannelMin(RCRadio::Channel channel);

	// Gets the maximum pulse width a channel can return
	static long getChannelMax(RCRadio::Channel channel);
};

#endif

