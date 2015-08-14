#ifndef SLAM_TYPES_HPP
#define SLAM_TYPES_HPP

#include <sys/time.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <Eigen/Geometry>

namespace slam
{
	typedef double ScalarType;
	typedef Eigen::Matrix<ScalarType,3,1> Vector3;
	typedef Eigen::Transform<ScalarType,3,Eigen::Isometry> Transform;
	typedef Eigen::Matrix<ScalarType,6,6> Covariance;
	
	struct TransformWithCovariance
	{
		Transform transform;
		Covariance covariance;
	};
	
	/**
	 * @class Measurement
	 * @author Sebastian Kasperski
	 * @date 06/07/15
	 * @file Types.hpp
	 * @brief Base class for a single measurement or reading from a sensor.
	 * This can be a single laser scan, a point cloud, an image etc.
	 * It can either hold the actual data or contain a pointer to the data,
	 * in case that data management is handled separately from the mapping.
	 */
	class Measurement
	{
	public:
		Measurement(const timeval& time, const std::string& r, const std::string& s, const Transform& tr)
		:	mStamp(time), mRobotName(r), mSensorName(s),
			mUniqueID(boost::uuids::random_generator()()),
			mSensorPose(tr), mInverseSensorPose(tr.inverse()) {}
		
		virtual ~Measurement(){}
		
		timeval getTimestamp() const { return mStamp; }
		std::string getRobotName() const { return mRobotName; }
		std::string getSensorName() const { return mSensorName; }
		boost::uuids::uuid getUniqueID() const { return mUniqueID; }
		Transform getSensorPose() const { return mSensorPose; }
		Transform getInverseSensorPose() const { return mInverseSensorPose; }
		
	protected:
		timeval mStamp;
		std::string mRobotName;
		std::string mSensorName;
		boost::uuids::uuid mUniqueID;
		
		Transform mSensorPose;
		Transform mInverseSensorPose;
	};
}

#endif