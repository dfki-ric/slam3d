#include "LineScanSensor.hpp"

using namespace slam;

LineScanSensor::LineScanSensor(const std::string& n, Logger* l, const Transform& p)
 : Sensor(n, l, p)
{
	
}

LineScanSensor::~LineScanSensor()
{
	
}

TransformWithCovariance LineScanSensor::calculateTransform(Measurement* source, Measurement* target, Transform odometry) const
{
	// Transform guess in sensor frame
	Transform guess = source->getInverseSensorPose() * odometry * target->getSensorPose();
	
	// Cast to this sensors measurement type
	LineScanMeasurement* sourceCloud = dynamic_cast<LineScanMeasurement*>(source);
	LineScanMeasurement* targetCloud = dynamic_cast<LineScanMeasurement*>(target);
	if(!sourceCloud || !targetCloud)
	{
		mLogger->message(ERROR, "Measurement given to calculateTransform() is not a LineScan!");
		throw BadMeasurementType();
	}
	
	TransformWithCovariance twc;
	return twc;
}