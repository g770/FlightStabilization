// 
// 
// 

#include "MotorThrottleValue.h"
#include "Math.h"
#include "Motor.h"

void MotorThrottleValue::setThrottleValue(uint16_t value)
{
	this->throttleValue = value;
}

void MotorThrottleValue::increase(uint32_t increment)
{
	uint16_t newValue = Math::addWithoutOverflow(this->throttleValue, increment);

	if (newValue > Motor::MAX_THROTTLE_IN) {
		newValue = Motor::MAX_THROTTLE_IN;
	}

	this->throttleValue = newValue;
}

void MotorThrottleValue::decrease(uint32_t increment)
{
	uint16_t newValue = Math::subtractWithoutOverflow(this->throttleValue, increment);
	this->throttleValue = newValue;
}
