#include "CommonDefs.h"
#include "PWMReader.h"

// Constant used to mark an invalid type
const uint16_t INVALID_TIME = 0;

// Holds information associated with a individual pin
struct PinConfig
{
	TimeInterval* minPulse;
	TimeInterval* maxPulse;
};

// Max number of pins to monitor -- note, not all pins are supported in this range since
// the Zero doesn't support external interrupts on pin 4.
const uint8_t NUM_PINS = 8;

// Array of last observed pulse widths, indexed by pin number
static uint16_t pulseTimes[NUM_PINS] = { INVALID_TIME };

// Array of last pulse start times, indexed by pin number
static uint16_t startTimes[NUM_PINS] = { INVALID_TIME };

// The next three variables are set in the ISR to capture info about the pin change
static volatile uint16_t changedPinFlags;   // Bit vector of pins that have changed (lowest bit == pin 1)
static volatile uint8_t pinState[NUM_PINS];  // Array of pin state - HIGH or LOW
static volatile unsigned long timestamp[NUM_PINS];  // The timestamp of the pin change event

// Bit masks used with the changedPinFlags variable
static uint16_t flagMasks[NUM_PINS] = { 1 << 0, 1 << 1, 1 << 2, 1 << 3, 1 << 4, 1 << 5, 1 << 6, 1 << 7 };

// Array of config data for each pin -- used for bounds checking pin data
static PinConfig* pinInfo[NUM_PINS];

// Forward declaration of the ISR functions for each pin
static void handlePin1ChangeInterrupt();
static void handlePin2ChangeInterrupt();
static void handlePin3ChangeInterrupt();
static void handlePin5ChangeInterrupt();
static void handlePin6ChangeInterrupt();
static void handlePin7ChangeInterrupt();

// Map of pin number to the ISR for that pin -- note pins 0 and 4 are not supported.
typedef void(*isr_ptr_t)();
static isr_ptr_t interruptRoutines[] = { 0, handlePin1ChangeInterrupt, handlePin2ChangeInterrupt, handlePin3ChangeInterrupt, 0, handlePin5ChangeInterrupt, handlePin6ChangeInterrupt, handlePin7ChangeInterrupt };


void PWMReader::update()
{
	// Copy the volatile variable describing which pins changed
	uint16_t changedPins = changedPinFlags;

	// Iterate over the bit flags and see which pins need to be processed
	if (changedPins > 0)
	{
		for (uint8_t pinNum = 0; pinNum < NUM_PINS; pinNum++)
		{
			if (changedPins & flagMasks[pinNum])
			{
				// Copy the volatile pinstate and timestamp 
				noInterrupts();
				uint8_t pState = pinState[pinNum];
				unsigned long ts = timestamp[pinNum];
				interrupts();

				// If the pin state is high, a pulse is starting.  Save the 
				// start time and continue
				if (pState == HIGH)
				{
					startTimes[pinNum] = ts;
					pulseTimes[pinNum] = INVALID_TIME;
				}
				else
				{
					// A pulse ended, calculate the time
					pulseTimes[pinNum] = ts - startTimes[pinNum];
					//DEBUG_PRINT("Pulse time: ");
					//DEBUG_PRINTLN(pulseTimes[pinNum]);
				}

				// Clear the flag in the original bitmask
				//changedPinFlags = flagMasks[pinNum]
			}

		}

		changedPinFlags = 0;
	}
}


static inline void handlePinChangeInterrupt(uint16_t interruptedPin)
{
	// Set bitmask indicating which pin has changed.
	changedPinFlags |= flagMasks[interruptedPin];

	// Save the state of the pin (high/low)
	pinState[interruptedPin] = digitalRead(interruptedPin);

	// Save the timestamp of the change
	timestamp[interruptedPin] = micros();
}

static void handlePin1ChangeInterrupt()
{
	handlePinChangeInterrupt(1);
}

static void handlePin2ChangeInterrupt()
{
	handlePinChangeInterrupt(2);
}

static void handlePin3ChangeInterrupt()
{
	handlePinChangeInterrupt(3);
}

static void handlePin5ChangeInterrupt()
{
	handlePinChangeInterrupt(5);
}

static void handlePin6ChangeInterrupt()
{
	handlePinChangeInterrupt(6);
}

static void handlePin7ChangeInterrupt()
{
	handlePinChangeInterrupt(7);
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

	attachInterrupt(pinNum, interruptRoutines[pinNum], CHANGE);

	DEBUG_PRINT("PWMReader: monitoring pin ");
	DEBUG_PRINTLN(monitoredPin);
}

 bool PWMReader::getLastPulseWidth(TimeInterval* outInterval)
{
	uint8_t monitoredPin = this->getMonitoredPin();

	uint16_t pulseTime = pulseTimes[monitoredPin];

	TimeInterval pulseWidth = TimeInterval::CreateFromMicroseconds(pulseTime);
	if (pulseWidth >= *(pinInfo[monitoredPin]->minPulse) && pulseWidth <= *(pinInfo[monitoredPin]->maxPulse))
	{
		*outInterval = pulseWidth;
		return true;
	}

	return false;
}



