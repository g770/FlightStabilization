// PinConfiguration.h

#ifndef _PINCONFIGURATION_h
#define _PINCONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// Defins and configures the Arduino pins
class PinConfiguration
{

 public:
	 // Motor pinouts
	 // Rotation pattern:
	 // Top left: clockwise
	 // Top right: counter clockwise
	 // Bottom left: counter clockwise
	 // Bottom right: clockwise
	 static const uint8_t TOP_LEFT_MOTOR_PIN = 8;
	 static const uint8_t TOP_RIGHT_MOTOR_PIN = 9;
	 static const uint8_t BOTTOM_LEFT_MOTOR_PIN = 10;
	 static const uint8_t BOTTOM_RIGHT_MOTOR_PIN = 11;

	 // Receiver channel pins
	 static const uint8_t THROTTLE_PIN = 1;
	 static const uint8_t ROLL_PIN = 2;
	 static const uint8_t PITCH_PIN = 3;
	 static const uint8_t YAW_PIN = 5;

	 static void init();
};


#endif

