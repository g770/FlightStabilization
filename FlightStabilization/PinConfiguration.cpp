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
	pinMode(TOP_LEFT_MOTOR_PIN, OUTPUT);
	pinMode(TOP_RIGHT_MOTOR_PIN, OUTPUT);
	pinMode(BOTTOM_LEFT_MOTOR_PIN, OUTPUT);
	pinMode(BOTTOM_RIGHT_MOTOR_PIN, OUTPUT);

	pinMode(THROTTLE_PIN, INPUT);
	pinMode(ROLL_PIN, INPUT);
	pinMode(PITCH_PIN, INPUT);
	pinMode(YAW_PIN, INPUT);
}

