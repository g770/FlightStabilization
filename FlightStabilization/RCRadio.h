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
	 // min - The min value for the channel (typically degrees/sec)
	 // max - The max value for the channel
	void configureChannel(Channel channel, uint8_t pinNum, long min, long max);

	// Reads the current value of the given channel
	long readChannel(Channel channel);

private:
	struct ChannelConfig
	{
		long min;
		long max;
		PWMReader monitor;
	};

	static const int NUM_CHANNELS = 4;
	ChannelConfig pinMonitors[NUM_CHANNELS];
};


#endif

