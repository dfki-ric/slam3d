#ifndef SLAM_TYPES_HPP
#define SLAM_TYPES_HPP

#include <sys/time.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <Eigen/Geometry>

namespace slam3d
{
	typedef unsigned IdType;
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
	 * @class Indexer
	 * @brief Constructor for continuous identifiers.
	 */
	class Indexer
	{
	public:
		Indexer():mNextID(0) {}
		IdType getNext() { return mNextID++; }
	private:
		IdType mNextID;
	};
	
	/**
	 * @class Measurement
	 * @brief Base class for a single reading from a sensor.
	 * @details This can be a single laser scan, a point cloud, an image etc.
	 * It can either hold the actual data or contain a pointer to the data,
	 * in case that data management is handled separately from the mapping.
	 */
	class Measurement
	{
	public:
		Measurement(){}
		virtual ~Measurement(){}
		
		timeval getTimestamp() const { return mStamp; }
		std::string getRobotName() const { return mRobotName; }
		std::string getSensorName() const { return mSensorName; }
		boost::uuids::uuid getUniqueId() const { return mUniqueId; }
		Transform getSensorPose() const { return mSensorPose; }
		Transform getInverseSensorPose() const { return mInverseSensorPose; }
		
	protected:
		timeval mStamp;
		std::string mRobotName;
		std::string mSensorName;
		boost::uuids::uuid mUniqueId;
		
		Transform mSensorPose;
		Transform mInverseSensorPose;
	};
}

#endif