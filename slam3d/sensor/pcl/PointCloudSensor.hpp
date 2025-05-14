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

#pragma once

#include <slam3d/core/Graph.hpp>
#include <slam3d/core/ScanSensor.hpp>
#include <slam3d/core/PoseSensor.hpp>
#include <slam3d/sensor/pcl/RegistrationParameters.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/export.hpp>

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

		virtual const char* getTypeName() const { return "slam3d::PointCloudMeasurement"; }

	protected:
		PointCloud::Ptr mPointCloud;

	private:
		friend class boost::serialization::access;

		template <typename Archive>
		void serialize(Archive &ar, const unsigned int version)
		{
			// Tell boost::serialization that this is derived from Measurement.
			// It is required because we don't explicitely call Measurement::serialize()
			// from within PointCloudMeasurement::serialize().
			boost::serialization::void_cast_register<PointCloudMeasurement, Measurement>(
				static_cast<PointCloudMeasurement *>(NULL),
				static_cast<Measurement *>(NULL));
		}
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
		 * @brief Create a virtual measurement by accumulating pointclouds from given vertices.
		 * @param vertices list of vertices that should contain a PointCloudMeasurement
		 * @param pose origin of the accumulated pointcloud
		 * @throw BadMeasurementType
		 */		
		Measurement::Ptr createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const;
		
		/**
		 * @brief Create an ICP constraint between two point clouds.
		 * @param source
		 * @param target
		 * @param odometry
		 * @param loop whether this is a loop closure (true) or sequential match (false)
		 */
		virtual Constraint::Ptr createConstraint(const Measurement::Ptr& source,
		                                         const Measurement::Ptr& target,
		                                         const Transform& odometry,
		                                         bool loop);
		
		Transform align(PointCloudMeasurement::Ptr source,
						PointCloudMeasurement::Ptr target,
						const Transform& guess,
						const RegistrationParameters& config);

		/**
		 * @brief Sets parameters for the internal pointcloud registration.
		 * The standard set is always used to calculate the final transformation.
		 * A coarse set is used to initialize and verify loop-closures.
		 * @param param new configuration paramerters
		 * @param coarse whether to set coarse parameter set
		 */
		void setRegistrationParameters(const RegistrationParameters& param, bool coarse);
		
		/**
		 * @brief Set density of points in input scans.
		 * @param r
		 */
		void setScanResolution(double r);
		
		/**
		 * @brief Set density of points in accumulated map cloud.
		 * @param r
		 */
		void setMapResolution(double r);
		
		/**
		 * @brief Set parameters for outlier removal.
		 * @details An outlier is a point that has less then n neighbors within radius r.
		 * @param r
		 * @param n
		 */
		void setMapOutlierRemoval(double r, unsigned n);
		
		/**
		 * @brief Reduces the size of the source cloud by sampling with the given resolution.
		 * @param source
		 * @param resolution 
		 */
		static PointCloud::Ptr downsample(PointCloud::ConstPtr source, double resolution);
		
		/**
		 * @brief Reduces the size of the source cloud by sampling with internal scan resolution.
		 * @details This method might seem unneccessary and only exists so that ScanResolution
		 * can be an internal parameter of PointCloudSensor. It should be called before a scan is 
		 * given to addMeasurement().
		 * @param source
		 */
		PointCloud::Ptr downsampleScan(PointCloud::ConstPtr source);
		
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
		

		/**
		 * @brief configuration struct for buildMap
		 * 
		 */
		struct MapConfig {
			MapConfig():removeOutliers(true),downsample(true),cut(false) {}
			bool removeOutliers;
			bool downsample;
			bool cut;
			PointType min;
			PointType max;
		};
		/**
		 * @brief Build an accumulated point cloud map from given vertices.
		 * @param vertices
		 */
		PointCloud::Ptr buildMap(const VertexObjectList& vertices, const MapConfig& mapConfig = MapConfig()) const;

		/**
		 * @brief backward-compatibility version
		 */
		PointCloud::Ptr buildMap(const VertexObjectList& vertices, bool doRemoveOutliers, bool doDownsample = true) const;
		
		/**
		 * @brief Fill ground plane around center.
		 * @details Estimates a ground plane within the given cloud using RANSAC 
		 * and fills this plane with additional points within radius around the
		 * origin. If no ground plane exists in the scan, the result is
		 * undefined, e.g. RANSAC will just return any plane in the scan.
		 * @param cloud
		 * @param radius
		 */
		void fillGroundPlane(PointCloud::Ptr cloud, ScalarType radius);

	protected:
		RegistrationParameters mFineConfiguration;
		RegistrationParameters mCoarseConfiguration;
		
		double   mScanResolution;
		double   mMapResolution;
		double   mMapOutlierRadius;
		unsigned mMapOutlierNeighbors;
	};
}

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive & ar, pcl::PCLPointField & f, const unsigned int version)
		{
			ar & f.count;
			ar & f.datatype;
			ar & f.name;
			ar & f.offset;
		}

		template<class Archive>
		void serialize(Archive & ar, pcl::PCLHeader & h, const unsigned int version)
		{
			ar & h.frame_id;
			ar & h.seq;
			ar & h.stamp;
		}

		template<class Archive>
		void serialize(Archive & ar, Eigen::Vector4f &point, const unsigned int version)
		{
			ar & point[0];
			ar & point[1];
			ar & point[2];
			ar & point[3];
		}

		template<class Archive>
		void serialize(Archive & ar, Eigen::Quaternionf &point, const unsigned int version)
		{
			ar & point.x();
			ar & point.y();
			ar & point.z();
			ar & point.w();
		}

		template<class Archive>
		void serialize(Archive & ar, slam3d::PointType &point, const unsigned int version)
		{
			ar & point.x;
			ar & point.y;
			ar & point.z;
		}

		template<class Archive>
		void serialize(Archive & ar, slam3d::PointCloud &cloud, const unsigned int version)
		{
			ar & cloud.header;
			ar & cloud.points;
			ar & cloud.width;
			ar & cloud.height;
			ar & cloud.is_dense;
			ar & cloud.sensor_origin_;
			ar & cloud.sensor_orientation_;
		}

		template<class Archive>
		inline void save_construct_data(Archive & ar, const slam3d::PointCloudMeasurement * m, const unsigned int file_version)
		{
			// save data required to construct instance
			ar << m->getPointCloud();
			ar << m->getRobotName();
			ar << m->getSensorName();
			ar << m->getSensorPose();
			ar << m->getUniqueId();
		}

		template<class Archive>
		inline void load_construct_data(Archive & ar, slam3d::PointCloudMeasurement * t, const unsigned int file_version)
		{
			// retrieve data from archive required to construct new instance
			slam3d::PointCloud::Ptr cloud;
			std::string robot;
			std::string sensor;
			slam3d::Transform pose;
			boost::uuids::uuid id;
			ar >> cloud;
			ar >> robot;
			ar >> sensor;
			ar >> pose;
			ar >> id;

			// invoke inplace constructor to initialize instance of PointCloudMeasurement
			::new(t)slam3d::PointCloudMeasurement(cloud, robot, sensor, pose, id);
		}
	}
}

BOOST_CLASS_EXPORT_KEY(slam3d::PointCloudMeasurement)
