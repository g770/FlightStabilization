// PWMReader.h

#ifndef _PWMREADER_h
#define _PWMREADER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "TimeInterval.h"

// Monitors pulses on a specific pin and 
// calculates the width of the pulse on that pin
class PWMReader
{
 public:
	 // Sets the pin to be monitored.  Assumes the pin has already
	 // been configured for input
	void monitorPin(uint8_t pinNum, TimeInterval minPulseWidth, TimeInterval maxPulseWidth);

	// Update run during main control loop
	static void update();

	// Gets the number of the monitored pin -- returns INVALID_PIN if 
	// no pin is currently monitored.
	uint8_t getMonitoredPin();

	// Gets the width of the last pulse.
	bool getLastPulseWidth(TimeInterval* );

	static const uint8_t INVALID_PIN = -1;

private:
	uint8_t monitoredPin;
};

#endif

