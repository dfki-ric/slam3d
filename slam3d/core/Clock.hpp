#pragma once

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

		/**
		 * @brief Returns time in seconds since t
		 * @param t
		 * @return time difference since t in seconds
		 */		
		double diff(struct timeval t)
		{
			timeval n = now();
			double dn = (double)(n.tv_sec + (double)n.tv_usec/1000000);
			double dt = (double)(t.tv_sec + (double)t.tv_usec/1000000);
			return dn - dt;
		}
	};
}
