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
		 */
		PointCloudMeasurement(const PointCloud::Ptr &cloud):mPointCloud(cloud){}

		timeval getTimestamp() const
		{
			timeval tv;
			// PCL header should contain microseconds
			tv.tv_sec  = mPointCloud->header.stamp / 1000000;
			tv.tv_usec = mPointCloud->header.stamp % 1000000;
			return tv;
		}

		virtual const char* getTypeName() const { return "slam3d::PointCloudMeasurement"; }

		/**
		 * @brief Gets the point cloud contained within this measurement.
		 * @return Constant shared pointer to the point cloud
		 */
		const PointCloud::Ptr getPointCloud() const {return mPointCloud;}

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
		}

		template<class Archive>
		inline void load_construct_data(Archive & ar, slam3d::PointCloudMeasurement * t, const unsigned int file_version)
		{
			// retrieve data from archive required to construct new instance
			slam3d::PointCloud::Ptr cloud;
			ar >> cloud;

			// invoke inplace constructor to initialize instance of PointCloudMeasurement
			::new(t)slam3d::PointCloudMeasurement(cloud);
		}
	}
}

BOOST_CLASS_EXPORT_KEY(slam3d::PointCloudMeasurement)
