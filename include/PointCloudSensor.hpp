#ifndef SLAM_POINTCLOUDSENSOR_HPP
#define SLAM_POINTCLOUDSENSOR_HPP

#include "Types.hpp"
#include "GICPConfiguration.hpp"
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
		 * @param r Name of the robot that made this measurement
		 * @param s Name of the sensor managing this measurement
		 */
		PointCloudMeasurement(const PointCloud::ConstPtr &cloud, const std::string& r, const std::string& s, const Transform& tr)
			:Measurement(timeval(), r, s, tr)
		{
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
		PointCloudSensor(const std::string& n, Logger* l, const Transform& p);
		
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
		TransformWithCovariance calculateTransform(Measurement* source, Measurement* target, Transform odometry) const;
		
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
		 * @brief Remove outliers from given pointcloud. A point is considered an outlier
		 * if it has less then min_neighbors within radius.
		 * @param source
		 * @param radius
		 * @param min_neighbors
		 */
		PointCloud::Ptr removeOutliers(PointCloud::ConstPtr source, double radius, unsigned min_neighbors) const;
		
		/**
		 * @brief Create a single point cloud that contains all measurements in vertices.
		 * The individual point clouds are transformed by their current pose in the graph,
		 * no additional alignement or optimazation is performed during this. The resulting
		 * point cloud is then resampled with the given resoltion.
		 * @param vertices
		 * @param resolution
		 */
		PointCloud::Ptr getAccumulatedCloud(VertexList vertices);
		
	protected:
		GICPConfiguration mConfiguration;
	};
}

#endif