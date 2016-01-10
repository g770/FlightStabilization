#include "RCRadio.h"

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
}


long RCRadio::readChannel(uint8_t channelNum)
{
	ChannelConfig channel = this->pinMonitors[channelNum];

	// Get the last pulse width measure on the pin for this channel
	uint8_t lastPulseWidth = channel.monitor.getLastPulseWidth();

	// Map the pulse width to the range for the channel and return it 
	return map(lastPulseWidth, MIN_CHANNEL_PULSE_WIDTH, MAX_CHANNEL_PULSE_WIDTH, channel.min, channel.max);
}

