
#include "PWMReader.h"
#include "PinChangeInt.h"

const uint8_t INVALID_TIME = -1;

static uint8_t monitoredPin = PWMReader::INVALID_PIN;

// Parallel vectors which store the start and end times indexed
// by pin numbers.  Currently these are hardcoded to a fixed number 
// of pins and the vectors are sparse.  TODO: Optimize to use 
// more compact data structures.
// (Note: tried using vectors here but hit type instantiation issues)
const uint8_t NUM_PINS = 50;
static volatile unsigned long startTimes[NUM_PINS];
static volatile unsigned long endTimes[NUM_PINS];

// Forward declarations
static void handleFallingInterrupt();

// The two callbacks below are called on rising and falling interrupts.  On the
// rising clock we record a start time and configure the falling signal to 
// trigger the other ISR function.
static void handleRisingInterrupt()
{
	uint8_t interruptedPin = PCintPort::arduinoPin;

	// Save the start time for this pin
	startTimes[interruptedPin] = micros();
	endTimes[interruptedPin] = INVALID_TIME;  // There is no valid end time yet

	// Attach the falling interrupt so we can capture the end time
	PCintPort::attachInterrupt(monitoredPin, &handleFallingInterrupt, FALLING);
}

static void handleFallingInterrupt()
{
	uint8_t interruptedPin = PCintPort::arduinoPin;

	// Save the end time of the interrupt
	endTimes[interruptedPin] = micros();

	// Reattach the handler to capture the next rising interrupt
	PCintPort::attachInterrupt(monitoredPin, &handleRisingInterrupt, RISING);
}

uint8_t PWMReader::getMonitoredPin()
{
	return monitoredPin;
}

void PWMReader::monitorPin(uint8_t pinNum)
{
	monitoredPin = pinNum;

	PCintPort::attachInterrupt(monitoredPin, &handleRisingInterrupt, RISING);

}

uint8_t PWMReader::getLastPulseWidth()
{
	unsigned long endTime = endTimes[this->getMonitoredPin()];
	unsigned long startTime = startTimes[this->getMonitoredPin()];

	if(endTime != INVALID_TIME && startTime != INVALID_TIME) 
	{ 
		// Return the duration of the pulse converted to milliseconds
		return (uint8_t)(endTime - startTime) / 1000;
	}
}



