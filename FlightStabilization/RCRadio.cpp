#include "RCRadio.h"
#include "CommonDefs.h"

// The min and max pulse widths that can be received on a channel
// The radio endpoints for each channel should be calibrated to send these as min and max values
// TODO: make const
TimeInterval MIN_CHANNEL_PULSE_WIDTH = TimeInterval::CreateFromMicroseconds(1000);
TimeInterval MAX_CHANNEL_PULSE_WIDTH = TimeInterval::CreateFromMicroseconds(2000);

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
	TimeInterval lastPulseWidth;
	bool channelReadResult = channelConfig.monitor.getLastPulseWidth(&lastPulseWidth);

	if (channelReadResult)
	{
		//DEBUG_PRINT("Pulse width: ");
		//DEBUG_PRINTLN(lastPulseWidth.getMicroSeconds());

		// Map the pulse width to the range for the channel and return it 
		if (channelConfig.scalingMin == NO_SCALING && channelConfig.scalingMax == NO_SCALING)
		{
			*result = lastPulseWidth.getMicroSeconds();
		}
		else
		{
			*result = map(lastPulseWidth.getMicroSeconds(), MIN_CHANNEL_PULSE_WIDTH.getMicroSeconds(), 
				MAX_CHANNEL_PULSE_WIDTH.getMicroSeconds(), channelConfig.scalingMin, channelConfig.scalingMax);
		}

		return true;
	}

	return false;
}

