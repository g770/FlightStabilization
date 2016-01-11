// RCRadio.h

#ifndef _RCRADIO_h
#define _RCRADIO_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <StandardCplusplus.h>
#include "PWMReader.h"

// Class that represents an RC radio.  It monitors pin inputs 
// representing RC channels and converts the pin inputs to channel values,
class RCRadio
{

 public:
	 static const int NO_SCALING = -1;

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
	long readChannel(Channel channel);

private:
	struct ChannelConfig
	{
		long scalingMin;
		long scalingMax;
		PWMReader monitor;
	};

	static const int NUM_CHANNELS = 4;
	ChannelConfig pinMonitors[NUM_CHANNELS];
};


#endif

