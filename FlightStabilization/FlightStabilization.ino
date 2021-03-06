/*
 Name:		FlightStabilization.ino
 Created:	1/10/2016 8:51:17 AM
 Author:	dwelzel
*/

#include "Deadband.h"
#include "IMUFilter.h"
#include "PID.h"
#include "Math.h"
#include "ChannelConfig.h"
#include <Wire.h>
#include "TimeInterval.h"
#include "PinConfiguration.h"
#include "QuadCopter.h"
#include <Servo.h>
#include "CommonDefs.h"
#include "Motor.h"
#include "QuadCopter.h"

// the setup function runs once when you press reset or power the board
#include "RCRadio.h"

// Main control loop time in microseconds
const uint32_t LOOP_TIME_USEC = 1000;

QuadCopter copter;

void setup() {

#ifdef DEBUG_LOG
	Serial.begin(115200);
#endif // DEBUG_LOG

	DEBUG_PRINTLN("Setup: waiting for esc power up");
	delay(5000);
	DEBUG_PRINTLN("Setup: waiting complete");


	// Setup pins
	PinConfiguration::init();
	DEBUG_PRINTLN("Setup: pins configured");


	copter.init();
	runTests();  // Will only be execute if TESTMODE is defined
}

// the loop function runs over and over again until power down or reset
void loop() {
	uint32_t start = micros();
	copter.update();
	uint32_t end = micros();

	// Handle the rare case that there is overflow in the micros() function
	uint32_t delayTime = 0;
	if (end <= start) {
		delayTime = LOOP_TIME_USEC;
	}
	else {
		delayTime = LOOP_TIME_USEC - (end - start);
	}

	delayMicroseconds(delayTime);
}

// Either compile in the actual test function or a stub
#ifdef TESTMODE
void runTests() {
	Serial.begin(9600);


}
#else
void runTests() { }
#endif