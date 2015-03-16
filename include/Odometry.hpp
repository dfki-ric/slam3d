#ifndef SLAM_ODOMETRY_HPP
#define SLAM_ODOMETRY_HPP

#include "Measurement.hpp"

namespace slam
{

	class Odometry
	{
	public:
		Odometry()
		{
		}
		~Odometry()
		{
		}
		
		virtual void getOdometricPose(Transform* tf, Covariance* cov) = 0;

	};

}

#endif // ODOMETRY_HPP
