// PWMReader.h

#ifndef _PWMREADER_h
#define _PWMREADER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// Class the monitors pulses on a specific pin and 
// calculates the width of the pulse
class PWMReader
{
 public:
	 // Sets the pin to be monitored.  Assumes the pin has already
	 // been configured for input
	void monitorPin(uint8_t pinNum);

	// Gets the number of the monitored pin -- returns INVALID_PIN if 
	// no pin is currently monitored.
	uint8_t getMonitoredPin();

	// Gets the width, in milliseconds of the last pulse.
	uint8_t getLastPulseWidth();

	static const uint8_t INVALID_PIN = -1;

private:
};

#endif

