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
		
		/**
		 * @brief Calculates covariance from simple motion model
		 * @param tf relative transform between two poses
		 * @return covariance of the relative transform tf
		 */
		virtual Covariance calculateCovariance(const Transform &tf) = 0;
		
	protected:
		Logger* mLogger;

	};

}

#endif
