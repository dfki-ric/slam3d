#ifndef SLAM_POINTCLOUDSENSOR_HPP
#define SLAM_POINTCLOUDSENSOR_HPP

#include "Types.hpp"
#include "GICPConfiguration.hpp"
#include "Sensor.hpp"
#include "GraphMapper.hpp"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace slam3d
{
#ifdef PCL_WITH_VIEWPOINT
	typedef pcl::PointWithViewpoint PointType;
#else
	typedef pcl::PointXYZ PointType;
#endif
	typedef pcl::PointCloud<PointType> PointCloud;
	
	/**
	 * @class PointCloudMeasurement
	 * @brief Specific Measurement of the PointCloudSensor. 
	 */
	class PointCloudMeasurement : public Measurement
	{
	public:
		typedef boost::shared_ptr<PointCloudMeasurement> Ptr;
	
	public:
		/**
		 * @brief Constructor from point cloud and sensor name.
		 * @param cloud shared pointer to the PointCloud
		 * @param r name of the robot that accquired this measurement
		 * @param s name of the sensor managing this measurement
		 * @param id unique identifier of this measurement
		 */
		PointCloudMeasurement(const PointCloud::Ptr &cloud,
		                      const std::string& r, const std::string& s,
		                      const Transform& tr, const boost::uuids::uuid id = boost::uuids::nil_uuid())
		{
			mPointCloud = cloud;
			mRobotName = r;
			mSensorName = s;
			mSensorPose = tr;
			mInverseSensorPose = tr.inverse();
			if(id.is_nil())
				mUniqueId = boost::uuids::random_generator()();
			else
				mUniqueId = id;

			// PCL header should contain microseconds
			mStamp.tv_sec  = cloud->header.stamp / 1000000;
			mStamp.tv_usec = cloud->header.stamp % 1000000;
		}
		
		/**
		 * @brief Gets the point cloud contained within this measurement.
		 * @return Constant shared pointer to the point cloud
		 */
		const PointCloud::Ptr getPointCloud() const {return mPointCloud;}
		
	protected:
		PointCloud::Ptr mPointCloud;
	};

	/**
	 * @class PointCloudSensor
	 * @brief Plugin for the mapper that manages point cloud measurements.
	 */
	class PointCloudSensor : public Sensor
	{
	public:
		/**
		 * @brief Constructor
		 * @param n unique name of this sensor (used to identify measurements)
		 * @param l pointer to a Logger to write messages
		 */
		PointCloudSensor(const std::string& n, Logger* l, const Transform& p);
		
		/**
		 * @brief Destructor
		 */
		~PointCloudSensor();
		
		/**
		 * @brief Estimates the 6DoF transformation between source and target point cloud
		 * @details It applies the Generalized Iterative Closest Point algorithm. (GICP)
		 * @param source
		 * @param target
		 */
		TransformWithCovariance calculateTransform(Measurement::Ptr source, Measurement::Ptr target, Transform odometry, bool coarse = false) const;
				
		/**
		 * @brief Create a virtual measurement by accumulating pointclouds from given vertices.
		 * @param vertices list of vertices that should contain a PointCloudMeasurement
		 * @param pose origin of the accumulated pointcloud
		 * @throw BadMeasurementType
		 */		
		Measurement::Ptr createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const;
		
		/**
		 * @brief Sets configuration for fine GICP algorithm.
		 * @param c New configuration paramerters
		 */
		void setFineConfiguaration(GICPConfiguration c) { mFineConfiguration = c; }
		
		/**
		 * @brief Sets configuration for coarse GICP algorithm.
		 * @param c New configuration paramerters
		 */
		void setCoarseConfiguaration(GICPConfiguration c) { mCoarseConfiguration = c; }
		
		/**
		 * @brief Reduces the size of the source cloud by sampling with the given resolution.
		 * @param source
		 * @param resolution 
		 */
		PointCloud::Ptr downsample(PointCloud::ConstPtr source, double resolution) const;
		
		/**
		 * @brief Transform source cloud by given transformation.
		 * @param source
		 * @param tf
		 */
		PointCloud::Ptr transform(PointCloud::ConstPtr source, const Transform tf) const;
		
		/**
		 * @brief Removes outliers from given pointcloud.
		 * @details A point is considered an outlier if it has less then min_neighbors within radius.
		 * @param source
		 * @param radius
		 * @param min_neighbors
		 */
		PointCloud::Ptr removeOutliers(PointCloud::ConstPtr source, double radius, unsigned min_neighbors) const;
		
		/**
		 * @brief Creates a single point cloud that contains all measurements in vertices.
		 * @details The individual point clouds are transformed by their current pose in the graph,
		 * no additional alignement or optimization is performed during this.
		 * @param vertices
		 * @return accumulated pointcloud
		 * @throw BadMeasurementType
		 */
		PointCloud::Ptr getAccumulatedCloud(const VertexObjectList& vertices) const;
		
	protected:
		GICPConfiguration mFineConfiguration;
		GICPConfiguration mCoarseConfiguration;
	};
}

#endif