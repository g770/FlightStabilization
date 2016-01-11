#include "RCRadio.h"
#include "CommonDefs.h"

// The min and max pulse widths on a channel
const long MIN_CHANNEL_PULSE_WIDTH = 1000;
const long MAX_CHANNEL_PULSE_WIDTH = 2000;

void RCRadio::configureChannel(Channel channel, uint8_t pinNum, long min, long max)
{
	// Instantite an object that will handle reading the values of the 
	// pin and calculate the pulse width
	PWMReader reader;
	reader.monitorPin(pinNum);

	// Store the configuration for this channel
	ChannelConfig config;
	config.min = min;
	config.max = max;
	config.monitor = reader;

	this->pinMonitors[channel] = config;

	DEBUG_PRINT("RCRadio: Configured channel ");
	DEBUG_PRINT(channel);
	DEBUG_PRINT(" on pin ");
	DEBUG_PRINTLN(pinNum);
}


long RCRadio::readChannel(Channel channel)
{
	ChannelConfig channelConfig = this->pinMonitors[channel];

	// Get the last pulse width measure on the pin for this channel
	uint8_t lastPulseWidth = channelConfig.monitor.getLastPulseWidth();

	DEBUG_PRINT("RCRadio: Read pulse width on channel ");
	DEBUG_PRINT(channel);
	DEBUG_PRINT(": ");
	DEBUG_PRINTLN(lastPulseWidth);

	// Map the pulse width to the range for the channel and return it 
	return map(lastPulseWidth, MIN_CHANNEL_PULSE_WIDTH, MAX_CHANNEL_PULSE_WIDTH, channelConfig.min, channelConfig.max);
}

