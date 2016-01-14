#include "RCRadio.h"
#include "CommonDefs.h"

// The min and max pulse widths that can be received on a channel
const long MIN_CHANNEL_PULSE_WIDTH = 1000;
const long MAX_CHANNEL_PULSE_WIDTH = 2000;

void RCRadio::configureChannel(Channel channel, uint8_t pinNum, long scalingMin, long scalingMax)
{
	// Instantite an object that will handle reading the values of the 
	// pin and calculate the pulse width
	PWMReader reader;
	reader.monitorPin(pinNum, MIN_CHANNEL_PULSE_WIDTH, MAX_CHANNEL_PULSE_WIDTH);

	// Store the configuration for this channel
	ChannelConfig config;
	config.scalingMin = scalingMin;
	config.scalingMax = scalingMax;
	config.monitor = reader;

	this->pinMonitors[channel] = config;

	DEBUG_PRINT("RCRadio: Configured channel ");
	DEBUG_PRINT(channel);
	DEBUG_PRINT(" on pin ");
	DEBUG_PRINTLN(pinNum);
}


bool RCRadio::readChannel(Channel channel, long* result)
{
	ChannelConfig channelConfig = this->pinMonitors[channel];

	// Get the last pulse width measure on the pin for this channel
	uint16_t lastPulseWidth = channelConfig.monitor.getLastPulseWidth();

	if (lastPulseWidth >= MIN_CHANNEL_PULSE_WIDTH && lastPulseWidth <= MAX_CHANNEL_PULSE_WIDTH)
	{
		DEBUG_PRINT("RCRadio: Read pulse width on channel ");
		DEBUG_PRINT(channel);
		DEBUG_PRINT(": ");
		DEBUG_PRINTLN(lastPulseWidth);

		// Map the pulse width to the range for the channel and return it 
		if (channelConfig.scalingMin == NO_SCALING && channelConfig.scalingMax == NO_SCALING)
		{
			*result = lastPulseWidth;
		}
		else
		{
			*result = map(lastPulseWidth, MIN_CHANNEL_PULSE_WIDTH, MAX_CHANNEL_PULSE_WIDTH, channelConfig.scalingMin, channelConfig.scalingMax);
		}

		return true;
	}

	return false;
}

