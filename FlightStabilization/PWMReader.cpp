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

const uint8_t NUM_PINS = 14;
static uint16_t pulseTimes[NUM_PINS] = { INVALID_TIME };
static uint16_t startTimes[NUM_PINS] = { INVALID_TIME };

static volatile uint16_t changedPinFlags;
static volatile uint8_t pinState[NUM_PINS];
static volatile unsigned long timestamp[NUM_PINS];

static uint16_t flagMasks[NUM_PINS] = { 1 << 0, 1 << 1, 1 << 2, 1 << 3, 1 << 4, 1 << 5, 1 << 6, 1 << 7, 1 << 8, 1 << 9, 1 << 10, 1 << 11, 1 << 12, 1 << 13 };

// Array of config data for each pin -- used for bounds checking pin data
static PinConfig* pinInfo[NUM_PINS];

void PWMReader::update()
{
	// Copy the volatile variable describing which pins changed
	uint8_t changedPins = changedPinFlags;

	// Iterate over the bit flags and see which pins need to be processed
	for (uint8_t pinNum = 0; pinNum < NUM_PINS; pinNum++)
	{
		if (changedPinFlags & flagMasks[pinNum])
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
			}
		}
	}
}


static void handlePinChangeInterrupt()
{
	// Set bitmask indicating which pin has changed.
	uint16_t interruptedPin = PCintPort::arduinoPin;
	changedPinFlags |= flagMasks[interruptedPin];

	// Save the state of the pin (high/low)
	pinState[interruptedPin] = digitalRead(interruptedPin);

	// Save the timestamp of the change
	timestamp[interruptedPin] = micros();
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

	PCintPort::attachInterrupt(monitoredPin, &handlePinChangeInterrupt, CHANGE);

	DEBUG_PRINT("PWMReader: monitoring pin ");
	DEBUG_PRINTLN(monitoredPin);
}

TimeInterval PWMReader::getLastPulseWidth()
{
	uint8_t monitoredPin = this->getMonitoredPin();

	uint16_t pulseTime = pulseTimes[monitoredPin];

	TimeInterval pulseWidth = TimeInterval::CreateFromMicroseconds(pulseTime);
	return pulseWidth;

	/*
	if (pulseWidth >= *(pinInfo[monitoredPin]->minPulse) && pulseWidth <= *(pinInfo[monitoredPin]->maxPulse))
	{
	}
	*/

}



