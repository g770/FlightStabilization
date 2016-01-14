/*
 Name:		FlightStabilization.ino
 Created:	1/10/2016 8:51:17 AM
 Author:	dwelzel
*/

#include "TimeInterval.h"
#include "PinConfiguration.h"
#include "QuadCopter.h"
#include <Servo.h>
#include "CommonDefs.h"
#include "Motor.h"
#include "QuadCopter.h"

// the setup function runs once when you press reset or power the board
#include "RCRadio.h"
#include <utility.h>
#include <unwind-cxx.h>
#include <system_configuration.h>

QuadCopter copter;

void setup() {

#ifdef DEBUG_LOG
	Serial.begin(9600);
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