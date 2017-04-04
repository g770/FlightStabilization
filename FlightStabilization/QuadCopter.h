// QuadCopter.h

#ifndef _QUADCOPTER_h
#define _QUADCOPTER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "RCRadio.h"
#include "Motor.h"
#include "PID.h"
#include "IMUFilter.h"
#include "MotorThrottleValue.h"

// Core class that represenst a quadcopter and all of its components.
class QuadCopter
{

 public:
	 // One time intialization of the quadcopter
	 void init();

	 // Core update loop, called during the main update loop.  This processes inputs from the receiver, 
	 // uses the PID controller and sends the appropriate outputs to the motors.
	 void update();

 private:
	 void processThottleChannel(RCRadio::ChannelData &);
	 void processPitchChannel(RCRadio::ChannelData &, imu::Vector<3>&);
	 void processRollChannel(RCRadio::ChannelData &, imu::Vector<3>&);
	 void processYawChannel(RCRadio::ChannelData &, imu::Vector<3>&);
	 void processArmingCommand(RCRadio::ChannelData &);
	 void processDisarmingCommand(RCRadio::ChannelData &);
	 void armMotors();
	 void disarmMotors();
	 bool isInArmingCommandErrorBounds(long, long) const;

	 static const int NUM_MOTORS = 4;

	 RCRadio receiver;
	 Motor motors[NUM_MOTORS];
	 IMUFilter imu;
	 PID throttlePID;
	 PID rollPID;
	 PID pitchPID;
	 PID yawPID;
	 MotorThrottleValue topLeftMotorValue;	 
	 MotorThrottleValue topRightMotorValue;
	 MotorThrottleValue bottomLeftMotorValue;
	 MotorThrottleValue bottomRightMotorValue;
	 bool isArmed;

	 static const int ARMING_COUNT = 20;
	 long throttleTotal, pitchTotal, rollTotal, yawTotal;
};


#endif

