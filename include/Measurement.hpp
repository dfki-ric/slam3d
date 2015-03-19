#ifndef SLAM_MEASUREMENT_HPP
#define SLAM_MEASUREMENT_HPP

#include <sys/time.h>
#include <Eigen/Geometry>

namespace slam
{
	typedef Eigen::Isometry3d Transform;
	typedef Eigen::Matrix<double,6,6> Covariance;
	
	struct TransformWithCovariance
	{
		Transform transform;
		Covariance covariance;
	};
	
	class Measurement
	{
	public:
		Measurement(unsigned int id, timeval t, std::string s)
			:mID(id), mStamp(t), mSensorName(s){}
		
		unsigned int getID() { return mID; }
		timeval getTimestamp() { return mStamp; }
		std::string getSensorName() { return mSensorName; }
		
	private:
		unsigned int mID;
		timeval mStamp;
		std::string mSensorName;
	};
}

#endif