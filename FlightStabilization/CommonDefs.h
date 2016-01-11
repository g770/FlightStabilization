#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

// If this is define, tests are executed and test
// code is included
//#define TESTMODE

// Define to enable debug logging to the Serial console
#define DEBUG_LOG

#ifdef DEBUG_LOG
#define DEBUG_PRINT(str) Serial.print(str)
#define DEBUG_PRINTLN(str) Serial.println(str)
#else
#define DEBUG_PRINT(str) 
#define DEBUG_PRINTLN(str) 
#endif

#endif // COMMON_DEFS_H
