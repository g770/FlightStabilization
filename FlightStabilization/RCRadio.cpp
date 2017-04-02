#include "RCRadio.h"
#include "CommonDefs.h"
#include "Math.h"
#include "ChannelConfig.h"

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

void RCRadio::readChannels(ChannelData &result) const
{
	for (int channel = 0; channel < NUM_CHANNELS; channel++)
	{
		double data;
		bool readResult = this->readChannel((Channel)channel, &data);
		result.channelData[channel] = data;
		result.channelResults[channel] = readResult;
	}
}

bool RCRadio::readChannel(Channel channel, double* result) const
{
	ChannelConfig channelConfig = this->pinMonitors[channel];

	// Get the last pulse width measure on the pin for this channel
	TimeInterval lastPulseWidth;
	bool channelReadResult = channelConfig.monitor.getLastPulseWidth(&lastPulseWidth);

	if (channelReadResult)
	{
		// Process deadband
		uint32_t msec = processDeadband(channel, lastPulseWidth);

		// Map the pulse width to the range for the channel and return it 
		if (channelConfig.scalingMin == NO_SCALING && channelConfig.scalingMax == NO_SCALING)
		{
			*result = msec;
		}
		else
		{

			*result = Math::map_double(msec, MIN_CHANNEL_PULSE_WIDTH.getMicroSeconds(),
				MAX_CHANNEL_PULSE_WIDTH.getMicroSeconds(), channelConfig.scalingMin, channelConfig.scalingMax);
		}

		return true;
	}

	return false;
}


uint32_t RCRadio::processDeadband(Channel channel, TimeInterval& pulseWidth) const
{
	const Deadband* deadband = ::ChannelConfig::getChannelDeadband(channel);
	uint32_t msec = pulseWidth.getMicroSeconds();
	uint32_t retVal = msec;

	if (deadband != NULL && msec >= deadband->lowEnd && msec <= deadband->highEnd)
	{
		retVal = deadband->deadbandValue;
	}

	return retVal;
}

