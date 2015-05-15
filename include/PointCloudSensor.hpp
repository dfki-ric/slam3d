#ifndef SLAM_POINTCLOUDSENSOR_HPP
#define SLAM_POINTCLOUDSENSOR_HPP

#include "Types.hpp"
#include "Sensor.hpp"
#include "GraphMapper.hpp"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace slam
{
	typedef pcl::PointXYZI PointType;
	typedef pcl::PointCloud<PointType> PointCloud;
	
	/**
	 * @class PointCloudMeasurement
	 * @author Sebastian Kasperski
	 * @date 04/17/15
	 * @file PointCloudSensor.hpp
	 * @brief Measurement of the PointCloudSensor. 
	 */
	class PointCloudMeasurement : public Measurement
	{
	public:
		/**
		 * @brief Constructor from point cloud and sensor name
		 * @param cloud Shared pointer to the PointCloud
		 * @param s Name of the sensor managing this measurement
		 */
		PointCloudMeasurement(const PointCloud::ConstPtr &cloud, std::string s)
		{
			mSensorName = s;
			mPointCloud = cloud;

			// PCL header should contain microseconds
			mStamp.tv_sec  = cloud->header.stamp / 1000000;
			mStamp.tv_usec = cloud->header.stamp % 1000000;
		}
		
		/**
		 * @brief Get the point cloud contained within this measurement.
		 * @return Constant shared pointer to the point cloud
		 */
		const PointCloud::ConstPtr getPointCloud() const {return mPointCloud;}
		
	protected:
		PointCloud::ConstPtr mPointCloud;
	};

	/**
	 * @class GICPConfiguration
	 * @author Sebastian Kasperski
	 * @date 04/17/15
	 * @file PointCloudSensor.hpp
	 * @brief Collection of configuration parameters for the "Generalized
	 * Iterative Closest Point" algorithm used for alignement of collected
	 * point clouds.
	 */
	struct GICPConfiguration
	{
		double max_correspondence_distance;
		int maximum_iterations;
		double transformation_epsilon;
		double euclidean_fitness_epsilon;
		int correspondence_randomness;
		int maximum_optimizer_iterations;
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
		                      point_cloud_density(0.2), max_fitness_score(2.0),
		                      position_sigma(0.001), orientation_sigma(0.0001), max_sensor_distance(2.0) {};
	};

	/**
	 * @class PointCloudSensor
	 * @author Sebastian Kasperski
	 * @date 04/17/15
	 * @file PointCloudSensor.hpp
	 * @brief Plugin for the mapper that manages point cloud measurements.
	 */
	class PointCloudSensor : public Sensor
	{
	public:
		/**
		 * @brief Constructor
		 * @param n Unique name of this sensor (used to identify measurements)
		 * @param l Pointer to a Logger to write messages
		 */
		PointCloudSensor(std::string n, Logger* l);
		
		/**
		 * @brief Destructor
		 */
		~PointCloudSensor();
		
		/**
		 * @brief Estimates the 6DoF transformation and  between source and target point cloud
		 * by applying the Generalized Iterative Closest Point algorithm. (GICP)
		 * @param source
		 * @param target
		 */
		TransformWithCovariance calculateTransform(Measurement* source, Measurement* target, Transform guess) const;
		
		/**
		 * @brief Set configuration for GICP algorithm
		 * @param c New configuration paramerters
		 */
		void setConfiguaration(GICPConfiguration c) { mConfiguration = c; }
		
		/**
		 * @brief Reduces the size of the source cloud by sampling with the given resolution.
		 * @param source
		 * @param resolution 
		 */
		PointCloud::Ptr downsample(PointCloud::ConstPtr source, double resolution) const;
		
		/**
		 * @brief Create a single point cloud that contains all measurements in vertices.
		 * The individual point clouds are transformed by their current pose in the graph,
		 * no additional alignement or optimazation is performed during this. The resulting
		 * point cloud is then resampled with the given resoltion.
		 * @param vertices
		 * @param resolution
		 */
		PointCloud::Ptr getAccumulatedCloud(VertexList vertices, double resolution);
		
	protected:
		GICPConfiguration mConfiguration;
	};
}

#endif