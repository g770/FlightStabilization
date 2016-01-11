// 
// 
// 

#include "CommonDefs.h"
#include "PinConfiguration.h"

static const int NUM_PINS = 8;
static uint8_t allPins[] = 
{ 
	PinConfiguration::TOP_LEFT_MOTOR_PIN, 
	PinConfiguration::TOP_RIGHT_MOTOR_PIN, 
	PinConfiguration::BOTTOM_LEFT_MOTOR_PIN, 
	PinConfiguration::BOTTOM_RIGHT_MOTOR_PIN,
	PinConfiguration::THROTTLE_PIN, 
	PinConfiguration::ROLL_PIN, 
	PinConfiguration::PITCH_PIN, 
	PinConfiguration::YAW_PIN 
};

void PinConfiguration::init()
{

	for (int i = 0; i < NUM_PINS; i++)
	{
		pinMode(allPins[i], OUTPUT);
		DEBUG_PRINT("PinConfiguration: Configured pin for output - ");
		DEBUG_PRINTLN(allPins[i]);
	}
}

