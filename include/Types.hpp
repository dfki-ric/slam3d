#ifndef SLAM_TYPES_HPP
#define SLAM_TYPES_HPP

#include <sys/time.h>
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
		Measurement(){}
		Measurement(timeval t, std::string s)
			:mStamp(t), mSensorName(s){}
		virtual ~Measurement(){}
		
		timeval getTimestamp() const { return mStamp; }
		std::string getSensorName() const { return mSensorName; }
		
	protected:
		timeval mStamp;
		std::string mSensorName;
	};
}

#endif