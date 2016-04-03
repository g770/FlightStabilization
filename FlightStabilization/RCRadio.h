// RCRadio.h

#ifndef _RCRADIO_h
#define _RCRADIO_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//#include <StandardCplusplus.h>
#include "PWMReader.h"

// Class that represents an RC radio.  It monitors pin inputs 
// representing RC channels and converts the pin inputs to channel values,
class RCRadio
{

 public:
	 static const int NO_SCALING = -1;
	 static const int NUM_CHANNELS = 4;

	 // Structure to hold the results of reading all channels.  The 
	 // arrays are indexed by the Channel enum.
	 struct ChannelData
	 {
		 // The data for each channel
		 long channelData[NUM_CHANNELS];

		 // Bool indicating if the channel contains valid data
		 bool channelResults[NUM_CHANNELS];
	 };

	 // Channels supported by the radio
	 enum Channel 
	 {
		 THROTTLE = 0,
		 ROLL,
		 PITCH,
		 YAW
	 };

	 // Configures a channel to monitor
	 // channelNum - The RC channel number
	 // pinNum - The pin number that is receiving the channel input
	 // scalingMin, scalingMax -- Defines the range that results will be 
	 // mapped into when reading from the channel.  To disable scaling 
	 // on a channel, pass NO_SCALING for both values
	void configureChannel(Channel channel, uint8_t pinNum, long scalingMin, long scalingMax);

	// Reads the current value of the given channel
	bool readChannel(Channel channel, long* result);

	// Reads all of the channels into a single structure
	void readChannels(ChannelData &result);

private:
	struct ChannelConfig
	{
		long scalingMin;
		long scalingMax;
		PWMReader monitor;
	};

	ChannelConfig pinMonitors[NUM_CHANNELS];
};


#endif

