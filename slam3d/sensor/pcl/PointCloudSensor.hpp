// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2017 S. Kasperski
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SLAM_POINTCLOUDSENSOR_HPP
#define SLAM_POINTCLOUDSENSOR_HPP

#include <slam3d/sensor/pcl/GICPConfiguration.hpp>

#include <slam3d/core/Graph.hpp>
#include <slam3d/core/ScanSensor.hpp>
#include <slam3d/core/PoseSensor.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace slam3d
{
	typedef pcl::PointXYZ PointType;
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
		 * @param p pose of the sensor in the robot's coordinate frame
		 * @param id unique identifier of this measurement
		 */
		PointCloudMeasurement(const PointCloud::Ptr &cloud,
		                      const std::string& r, const std::string& s,
		                      const Transform& p, const boost::uuids::uuid id = boost::uuids::nil_uuid())
		: Measurement(r, s, p, id)
		{
			mPointCloud = cloud;

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
	class PointCloudSensor : public ScanSensor
	{
	public:
		/**
		 * @brief Constructor
		 * @param n unique name of this sensor (used to identify measurements)
		 * @param l pointer to a Logger to write messages
		 */
		PointCloudSensor(const std::string& n, Logger* l);
		
		/**
		 * @brief Destructor
		 */
		~PointCloudSensor();
		
		/**
		 * @brief Set if scan matching to neighbors is done in a separate thread.
		 * @param mt 
		 */
		void setMultiThreaded(bool mt) { mMultiThreaded = mt; }
		
		/**
		 * @brief Add a new measurement from this sensor.
		 * @param cloud
		 * @param force
		 */
		bool addMeasurement(const PointCloudMeasurement::Ptr& cloud, bool force = false);
		
		/**
		 * @brief Add a new measurement from this sensor together with an odometry pose.
		 * @param cloud
		 * @param odom
		 * @param force
		 */
		bool addMeasurement(const PointCloudMeasurement::Ptr& cloud, const Transform& odom, bool force = false);
		
		/**
		 * @brief Calculate the estimated transform between two measurements of this sensor.
		 * @details It applies the Generalized Iterative Closest Point algorithm. (GICP)
		 * @param source measurement of the source node
		 * @param target measurement of the target node
		 * @param odometry estimation of robot movement
		 * @param coarse whether to do a coarse estimate
		 * @throw BadMeasurementType
		 */
		TransformWithCovariance calculateTransform(Measurement::Ptr source,
		                                           Measurement::Ptr target,
		                                           TransformWithCovariance odometry,
		                                           bool coarse = false) const;
				
		/**
		 * @brief Create a virtual measurement by accumulating pointclouds from given vertices.
		 * @param vertices list of vertices that should contain a PointCloudMeasurement
		 * @param pose origin of the accumulated pointcloud
		 * @throw BadMeasurementType
		 */		
		Measurement::Ptr createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const;
		
		/**
		 * @brief Create a linking constraint between source and target.
		 * @param source_id
		 * @param target_id
		 */
		void link(IdType source_id, IdType target_id);
		
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
		
		bool mMultiThreaded;
		
		Transform mLastOdometry;
		TransformWithCovariance mOdometryDelta;
	};
}

#endif
