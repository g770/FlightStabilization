#include "CommonDefs.h"
#include "PWMReader.h"
#include "PinChangeInt.h"

const uint16_t INVALID_TIME = 0;

struct PinConfig
{
	TimeInterval* minPulse;
	TimeInterval* maxPulse;
};

static uint8_t monitoredPin = PWMReader::INVALID_PIN;

// Parallel arrays which store the start times and pulse times indexed
// by pin numbers.  Currently these are hardcoded to a fixed number 
// of pins and the vectors are sparse.  TODO: Optimize to use 
// more compact data structures.
// (Note: tried using vectors here but hit type instantiation issues)
const uint8_t NUM_PINS = 25;
static volatile uint16_t pulseTimes[NUM_PINS] = { INVALID_TIME };
static volatile unsigned long startTimes[NUM_PINS] = { INVALID_TIME };

// Array of config data for each pin -- used for bounds checking pin data
static PinConfig* pinInfo[NUM_PINS];

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
		// Calculate the pulse width -- only store it if the value 
		// is within the expected range for this pin
		TimeInterval pulseWidth = TimeInterval::CreateFromMicroseconds(micros() - startTimes[interruptedPin]);
		
		if (pulseWidth >= *(pinInfo[interruptedPin]->minPulse) && pulseWidth <= *(pinInfo[interruptedPin]->maxPulse))
		{
			pulseTimes[interruptedPin] = pulseWidth.getMicroSeconds();
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

void PWMReader::monitorPin(uint8_t pinNum, TimeInterval minPulseWidth, TimeInterval maxPulseWidth)
{
	this->monitoredPin = pinNum;

	PinConfig* pConfig = new PinConfig();
	pConfig->minPulse = new TimeInterval(minPulseWidth);
	pConfig->maxPulse = new TimeInterval(maxPulseWidth);
	pinInfo[pinNum] = pConfig;

	PCintPort::attachInterrupt(monitoredPin, &handleRisingInterrupt, RISING);
	//attachInterrupt(digitalPinToInterrupt(pinNum), &handleRisingInterrupt, RISING);

	DEBUG_PRINT("PWMReader: monitoring pin ");
	DEBUG_PRINTLN(monitoredPin);
	DEBUG_PRINT("PWMReader: user function addr  ");
	DEBUG_PRINTLN((int)&handleRisingInterrupt);
}

TimeInterval PWMReader::getLastPulseWidth()
{
	return TimeInterval::CreateFromMicroseconds(pulseTimes[this->getMonitoredPin()]);
}



