#ifndef SLAM_ODOMETRY_HPP
#define SLAM_ODOMETRY_HPP

#include "Types.hpp"

namespace slam
{
	class Odometry
	{
	public:
		Odometry(){}
		~Odometry(){}
		
		/**
		 * @brief Get the robot's location at given point in time.
		 * @param stamp
		 */
		virtual Transform getOdometricPose(timeval stamp) = 0;
		
		/**
		 * @brief Get relative pose and uncertainty between two points in time.
		 * @param last
		 * @param next
		 */
		virtual TransformWithCovariance getRelativePose(timeval last, timeval next) = 0;

	};

}

#endif
