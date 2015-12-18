#ifndef SLAM_ODOMETRY_HPP
#define SLAM_ODOMETRY_HPP

#include "Types.hpp"
#include "Logger.hpp"

namespace slam3d
{
	/**
	 * @class OdometryException
	 * @brief Exception thrown when the requested odometry information is not available.
	 */
	class OdometryException: public std::exception
	{
	public:
		OdometryException(){}
		virtual const char* what() const throw()
		{
			return "Odometry at given time is not available!";
		}
	};
	
	/**
	 * @class Odometry
	 * @brief Base class for all odometry modules.
	 */
	class Odometry
	{
	public:

		Odometry(Logger* logger) : mLogger(logger) {}
		virtual ~Odometry(){}
		
		/**
		 * @brief Gets the robot's location at given poin in time.
		 * @param stamp
		 */
		virtual Transform getOdometricPose(timeval stamp) = 0;
		
		/**
		 * @brief Gets relative pose and uncertainty between two points in time.
		 * @param last
		 * @param next
		 * @return relative pose with covariance
		 * @throw OdometryException
		 */
		virtual TransformWithCovariance getRelativePose(timeval last, timeval next) = 0;
		
	protected:
		Logger* mLogger;

	};

}

#endif
