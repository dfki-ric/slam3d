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