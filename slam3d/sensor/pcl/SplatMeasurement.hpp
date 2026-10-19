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

#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <slam3d/sensor/pcl/SplatPoint.hpp>

namespace slam3d
{
	/**
	 * @class SplatMeasurement
	 * @brief Point cloud measurement that additionally carries 3D gaussian splats.
	 * @details The point cloud of the PointCloudMeasurement is used for registration and mapping
	 * (e.g. the centers of the opaque splats), so it does not have to match the splats point by point.
	 * The type name is the one of the PointCloudMeasurement, so it is used wherever point clouds are.
	 */
	class SplatMeasurement : public PointCloudMeasurement
	{
	public:
		typedef boost::shared_ptr<SplatMeasurement> Ptr;

	public:
		/**
		 * @brief Constructor from splats, point cloud and sensor name.
		 * @param splats the gaussian splats
		 * @param cloud point cloud used for registration and mapping
		 * @param r name of the robot that accquired this measurement
		 * @param s name of the sensor managing this measurement
		 * @param p pose of the sensor in the robot's coordinate frame
		 * @param id unique identifier of this measurement
		 */
		SplatMeasurement(const SplatCloud::Ptr &splats, const PointCloud::Ptr &cloud,
		                 const std::string& r, const std::string& s,
		                 const Transform& p, const boost::uuids::uuid id = boost::uuids::nil_uuid())
		: PointCloudMeasurement(cloud, r, s, p, id), mSplats(splats)
		{}

		/**
		 * @brief Gets the gaussian splats contained within this measurement.
		 */
		const SplatCloud::Ptr getSplats() const {return mSplats;}

		/**
		 * @brief Removes all splats with their center inside (or outside) the sphere, the point cloud is not changed.
		 * @param center sphere center in sensor coordinates
		 * @param radius sphere radius
		 * @param inside remove the splats inside the sphere if true, otherwise the ones outside
		 * @return number of removed splats
		 */
		size_t removeSplatsBySphere(const Eigen::Vector3d& center, double radius, bool inside = true);

	protected:
		SplatCloud::Ptr mSplats;

	private:
		friend class boost::serialization::access;

		template <typename Archive>
		void serialize(Archive &ar, const unsigned int version)
		{
			// all data is written by save_construct_data
			boost::serialization::void_cast_register<SplatMeasurement, PointCloudMeasurement>(
				static_cast<SplatMeasurement *>(NULL),
				static_cast<PointCloudMeasurement *>(NULL));
		}
	};
}

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive & ar, slam3d::SplatPoint &splat, const unsigned int version)
		{
			ar & splat.x;
			ar & splat.y;
			ar & splat.z;
			ar & splat.scale_0;
			ar & splat.scale_1;
			ar & splat.scale_2;
			ar & splat.rot_0;
			ar & splat.rot_1;
			ar & splat.rot_2;
			ar & splat.rot_3;
			ar & splat.opacity;
			ar & splat.f_dc_0;
			ar & splat.f_dc_1;
			ar & splat.f_dc_2;
		}

		template<class Archive>
		void serialize(Archive & ar, slam3d::SplatCloud &cloud, const unsigned int version)
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
		inline void save_construct_data(Archive & ar, const slam3d::SplatMeasurement * m, const unsigned int file_version)
		{
			ar << m->getSplats();
			ar << m->getPointCloud();
			ar << m->getRobotName();
			ar << m->getSensorName();
			ar << m->getSensorPose();
			ar << m->getUniqueId();
		}

		template<class Archive>
		inline void load_construct_data(Archive & ar, slam3d::SplatMeasurement * t, const unsigned int file_version)
		{
			slam3d::SplatCloud::Ptr splats;
			slam3d::PointCloud::Ptr cloud;
			std::string robot;
			std::string sensor;
			slam3d::Transform pose;
			boost::uuids::uuid id;
			ar >> splats;
			ar >> cloud;
			ar >> robot;
			ar >> sensor;
			ar >> pose;
			ar >> id;

			::new(t)slam3d::SplatMeasurement(splats, cloud, robot, sensor, pose, id);
		}
	}
}

BOOST_CLASS_EXPORT_KEY(slam3d::SplatMeasurement)
