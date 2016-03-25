/*
 Name:		FlightStabilization.ino
 Created:	1/10/2016 8:51:17 AM
 Author:	dwelzel
*/

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

QuadCopter copter;

void setup() {

	DEBUG_PRINTLN("Setup: waiting for esc power up");
	delay(10000);
	DEBUG_PRINTLN("Setup: waiting complete");

#ifdef DEBUG_LOG
	Serial.begin(115200);
#endif // DEBUG_LOG

	// Setup pins
	PinConfiguration::init();

	copter.init();
	runTests();  // Will only be execute if TESTMODE is defined
}

// the loop function runs over and over again until power down or reset
void loop() {
	copter.update();
}

// Either compile in the actual test function or a stub
#ifdef TESTMODE
void runTests() {
	Serial.begin(9600);


}
#else
void runTests() { }
#endif