#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/export.hpp>

#include <slam3d/core/Types.hpp>

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

		virtual const char* getTypeName() const override { return "slam3d::PointCloudMeasurement"; }

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
