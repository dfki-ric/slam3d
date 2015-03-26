#ifndef SLAM_POINTCLOUDSENSOR_HPP
#define SLAM_POINTCLOUDSENSOR_HPP

#include "Sensor.hpp"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace slam
{
	typedef pcl::PointXYZI PointType;
	typedef pcl::PointCloud<PointType> PointCloud;
	
	class PointCloudSensor : public Sensor
	{
	public:
		PointCloudSensor(std::string n, GraphMapper* m);
		~PointCloudSensor();
		
		void addPointCloud(PointCloud cloud);
		
	protected:
	};
}

#endif