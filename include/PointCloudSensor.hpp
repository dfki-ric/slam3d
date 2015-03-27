#ifndef SLAM_POINTCLOUDSENSOR_HPP
#define SLAM_POINTCLOUDSENSOR_HPP

#include "Measurement.hpp"
#include "Sensor.hpp"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace slam
{
	typedef pcl::PointXYZI PointType;
	typedef pcl::PointCloud<PointType> PointCloud;
	
	class PointCloudMeasurement : public Measurement
	{
	public:
		PointCloudMeasurement(const PointCloud& cloud, unsigned int id, std::string s)
		{
			mID = id;
			mSensorName = s;
			mPointCloud = cloud;

			// PCL header should contain microseconds
			mStamp.tv_sec  = cloud.header.stamp / 1000000;
			mStamp.tv_usec = cloud.header.stamp % 1000000;
		}
		
		const PointCloud* getPointCloud() const {return &mPointCloud;}
		
	protected:
		PointCloud mPointCloud;
	};

	struct GICPConfiguration
	{
		double max_correspondence_distance;
		unsigned maximum_iterations;
		double transformation_epsilon;
		double euclidean_fitness_epsilon;
		unsigned correspondence_randomness;
		unsigned maximum_optimizer_iterations;
		double rotation_epsilon;
		double point_cloud_density;
		double max_fitness_score;
		double position_sigma;
		double orientation_sigma;
		double max_sensor_distance;

		GICPConfiguration() : max_correspondence_distance(2.5),
		                      maximum_iterations(50), transformation_epsilon(1e-5),
		                      euclidean_fitness_epsilon(1.0), correspondence_randomness(20),
		                      maximum_optimizer_iterations(20), rotation_epsilon(2e-3),
		                      point_cloud_density(0.2), max_fitness_score(1.0),
		                      position_sigma(0.001), orientation_sigma(0.0001), max_sensor_distance(2.0) {};
	};

	class PointCloudSensor : public Sensor
	{
	public:
		PointCloudSensor(std::string n, GraphMapper* m, Logger* l);
		~PointCloudSensor();
		
		void setConfiguaration(GICPConfiguration c) { mConfiguration = c; }
		void addPointCloud(PointCloud cloud);
		
		TransformWithCovariance calculateTransform(PointCloud::ConstPtr source, PointCloud::ConstPtr target) const;
		
	protected:
		GICPConfiguration mConfiguration;
	};
}

#endif