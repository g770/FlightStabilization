// 
// 
// 

#include "TimeInterval.h"


TimeInterval TimeInterval::CreateFromMicroseconds(uint32_t value)
{
	return TimeInterval(value);
}

TimeInterval TimeInterval::CreateFromMilliseconds(uint32_t value)
{
	return TimeInterval(value * 1000);
}

uint32_t TimeInterval::getMicroSeconds()
{
	return this->value;
}

uint32_t TimeInterval::getMilliseconds()
{
	return this->value / 1000;
}
