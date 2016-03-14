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

