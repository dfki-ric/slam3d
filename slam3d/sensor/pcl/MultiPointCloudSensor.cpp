#include "MultiPointCloudSensor.hpp"

using namespace slam3d;

MultiPointCloudSensor::MultiPointCloudSensor(const std::string& n, Logger* l)
:PointCloudSensor(n, l)
{
	
}

bool MultiPointCloudSensor::createMultiMeasurement(
	const std::vector<PointCloudMeasurement::Ptr>& m, const std::vector<MetaData>& d,
	PointCloudMeasurement::Ptr& multi_m, MetaData& multi_d)
{
	if(m.size() != d.size())
	{
		mLogger->message(ERROR, "addSubMeasurement given vectors of different sizes");
		return false;
	}
	
	if(m.size() == 0)
	{
		mLogger->message(ERROR, "addSubMeasurement given empty vector");
		return false;
	}

	PointCloud::Ptr multiCloud(new PointCloud());

	timeval t;
	#pragma omp parallel for
	for(size_t i = 0; i < m.size(); i++)
	{
		PointCloud::Ptr tempCloud = transform(m[i]->getPointCloud(), d[i].sensorPose);

		#pragma omp critical 
		*multiCloud += *tempCloud;
		timeval cloudTime = m[i]->getTimestamp();
		if(timercmp(&cloudTime, &t, >)) t = cloudTime;
	}
	
	multi_m.reset(new PointCloudMeasurement(multiCloud));
	multi_d = initMetaData(t, multi_m->getTypeName(), d[0].robotName, mName, Transform::Identity());
	return true;
}
