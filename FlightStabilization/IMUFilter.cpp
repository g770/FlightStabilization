// 
// 
// 

#include "IMUFilter.h"

static const double error = 0.1;

imu::Vector<3> IMUFilter::getVector()
{
	imu::Vector<3> accelerometer = this->imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

	double x = accelerometer.x();
	if (abs(x) <= error) {
		x = 0.0;
	}

	double y = accelerometer.y();
	if (abs(x) <= error) {
		y = 0.0;
	}


	double z = accelerometer.z();
	if (abs(z) <= error) {
		z = 0.0;
	}

	return imu::Vector<3>(x, y, z);
}
