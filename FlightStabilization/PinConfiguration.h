// PinConfiguration.h

#ifndef _PINCONFIGURATION_h
#define _PINCONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class PinConfiguration
{

 public:
	 // Motor pinouts
	 static const uint8_t TOP_LEFT_MOTOR_PIN = 4;
	 static const uint8_t TOP_RIGHT_MOTOR_PIN = 4;
	 static const uint8_t BOTTOM_LEFT_MOTOR_PIN = 4;
	 static const uint8_t BOTTOM_RIGHT_MOTOR_PIN = 4;

	 // Receiver channel pins
	 static const uint8_t THROTTLE_PIN = 8;
	 static const uint8_t ROLL_PIN = 9;
	 static const uint8_t PITCH_PIN = 10;
	 static const uint8_t YAW_PIN = 11;

	 static void init();
};


#endif
