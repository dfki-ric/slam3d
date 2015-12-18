#ifndef SLAM_CLOCK_HPP
#define SLAM_CLOCK_HPP

#include <sys/time.h>

namespace slam3d
{
	/**
	 * @class Clock
	 * @brief Base class for different clock types.
	 * @details A clock is used to get timestamps for messages and samples.
	 * It might rely on system time, published clock from log files or simulated
	 * ticks from a simulation environment.
	 */
	class Clock
	{
	public:

		virtual ~Clock(){}
	
		/**
		 * @brief Returns current time depending on the clock.
		 * @return current timestamp
		 */
		virtual timeval now()
		{
			timeval tv;
			gettimeofday(&tv, 0);
			return tv;
		}
	};
}

#endif