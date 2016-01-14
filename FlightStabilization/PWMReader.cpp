#include "CommonDefs.h"
#include "PWMReader.h"
#include "PinChangeInt.h"

const uint16_t INVALID_TIME = 0;

struct PinConfig
{
	uint16_t minPulse;
	uint16_t maxPulse;
};

static uint8_t monitoredPin = PWMReader::INVALID_PIN;

// Parallel vectors which store the start and end times indexed
// by pin numbers.  Currently these are hardcoded to a fixed number 
// of pins and the vectors are sparse.  TODO: Optimize to use 
// more compact data structures.
// (Note: tried using vectors here but hit type instantiation issues)
const uint8_t NUM_PINS = 50;
static volatile uint16_t pulseTimes[NUM_PINS];
static volatile unsigned long startTimes[NUM_PINS];
static PinConfig pinInfo[NUM_PINS];

// Forward declarations
static void handleFallingInterrupt();

// The two callbacks below are called on rising and falling interrupts.  On the
// rising clock we record a start time and configure the falling signal to 
// trigger the other ISR function.
static void handleRisingInterrupt()
{
	uint8_t interruptedPin = PCintPort::arduinoPin;
	//uint8_t interruptedPin = 2;

	// Make sure the interrupted pin is valid
	if (interruptedPin < NUM_PINS)
	{
		// Save the start time for this pin
		startTimes[interruptedPin] = micros();
		pulseTimes[interruptedPin] = INVALID_TIME;
	}

	// Attach the falling interrupt so we can capture the end time
	PCintPort::attachInterrupt(interruptedPin, &handleFallingInterrupt, FALLING);
	//attachInterrupt(digitalPinToInterrupt(monitoredPin), &handleFallingInterrupt, FALLING);

	//DEBUG_PRINT("PWMReader: Rising interrupt on pin ");
	//DEBUG_PRINTLN(interruptedPin);
}

static void handleFallingInterrupt()
{
	uint8_t interruptedPin = PCintPort::arduinoPin;
	//uint8_t interruptedPin = 2;

	// Make sure the interrupted pin is valid
	if (interruptedPin < NUM_PINS)
	{
		// Calculate the pulse width
		uint16_t pulseWidth = micros() - startTimes[interruptedPin];
		if (pulseWidth >= pinInfo[interruptedPin].minPulse && pulseWidth <= pinInfo[interruptedPin].maxPulse)
		{
			pulseTimes[interruptedPin] = pulseWidth;
		}
	}

	// Reattach the handler to capture the next rising interrupt
	PCintPort::attachInterrupt(interruptedPin, &handleRisingInterrupt, RISING);
	//attachInterrupt(digitalPinToInterrupt(monitoredPin), &handleRisingInterrupt, RISING);

	//DEBUG_PRINT("PWMReader: Falling interrupt on pin ");
	//DEBUG_PRINTLN(interruptedPin);
}


uint8_t PWMReader::getMonitoredPin()
{
	return this->monitoredPin;
}

void PWMReader::monitorPin(uint8_t pinNum, uint16_t minPulseWidth, uint16_t maxPulseWidth)
{
	this->monitoredPin = pinNum;

	PinConfig pConfig;
	pConfig.minPulse = minPulseWidth;
	pConfig.maxPulse = maxPulseWidth;
	pinInfo[pinNum] = pConfig;

	PCintPort::attachInterrupt(monitoredPin, &handleRisingInterrupt, RISING);
	//attachInterrupt(digitalPinToInterrupt(pinNum), &handleRisingInterrupt, RISING);

	DEBUG_PRINT("PWMReader: monitoring pin ");
	DEBUG_PRINTLN(monitoredPin);
	DEBUG_PRINT("PWMReader: user function addr  ");
	DEBUG_PRINTLN((int)&handleRisingInterrupt);
}

uint16_t PWMReader::getLastPulseWidth()
{

	return pulseTimes[this->getMonitoredPin()];
}



