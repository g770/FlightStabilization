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

uint32_t TimeInterval::getValue()
{
	return this->value;
}

bool operator<=(TimeInterval & t1, TimeInterval & t2)
{
	return t1.getValue() <= t2.getValue();
}

bool operator>=(TimeInterval & t1, TimeInterval & t2)
{
	return t1.getValue() >= t2.getValue();
}
