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
	 static const uint8_t TOP_LEFT_MOTOR_PIN = 0;
	 static const uint8_t TOP_RIGHT_MOTOR_PIN = 0;
	 static const uint8_t BOTTOM_LEFT_MOTOR_PIN = 0;
	 static const uint8_t BOTTOM_RIGHT_MOTOR_PIN = 0;

	 // Receiver channel pins
	 static const uint8_t THROTTLE_PIN = 0;
	 static const uint8_t ROLL_PIN = 0;
	 static const uint8_t PITCH_PIN = 0;
	 static const uint8_t YAW_PIN = 0;

	 static void init();
};


#endif

