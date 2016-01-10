/*
 Name:		FlightStabilization.ino
 Created:	1/10/2016 8:51:17 AM
 Author:	dwelzel
*/

// If this is define, tests are executed and test
// code is included
#include <Servo.h>
#include "Motor.h"
#define TESTMODE

// the setup function runs once when you press reset or power the board
#include "RCRadio.h"
#include <utility.h>
#include <unwind-cxx.h>
#include <system_configuration.h>
void setup() {

	runTests();  // Will only be execute if TESTMODE is defined
}

// the loop function runs over and over again until power down or reset
void loop() {
  
}

// Either compile in the actual test function or a stub
#ifdef TESTMODE
void runTests() {
	Serial.begin(9600);


}
#else
void runTests() { }
#endif