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
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "PID.h"

class QuadCopter
{

 public:
	 void init();
	 void update();

 private:
	 void processThottleChannel(RCRadio::ChannelData &, uint16_t&, uint16_t&, uint16_t&, uint16_t&);
	 void processPitchChannel(RCRadio::ChannelData &, imu::Vector<3>&, uint16_t&, uint16_t&, uint16_t&, uint16_t&);
	 void processRollChannel(RCRadio::ChannelData &, imu::Vector<3>&, uint16_t&, uint16_t&, uint16_t&, uint16_t&);
	 void processYawChannel(RCRadio::ChannelData &, imu::Vector<3>&, uint16_t&, uint16_t&, uint16_t&, uint16_t&);
	 void processArmingCommand(RCRadio::ChannelData &);
	 void processDisarmingCommand(RCRadio::ChannelData &);
	 bool isInArmingCommandErrorBounds(long, long);

	 static const int NUM_MOTORS = 1;

	 RCRadio receiver;
	 Motor motors[NUM_MOTORS];
	 Adafruit_BNO055 imu;
	 PID throttlePID;
	 PID rollPID;
	 PID pitchPID;
	 PID yawPID;
	 bool isArmed;

	 static const int ARMING_COUNT = 150;
	 long throttleTotal, pitchTotal, rollTotal, yawTotal;
};


#endif

