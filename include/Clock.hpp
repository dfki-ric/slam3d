#ifndef SLAM_CLOCK_HPP
#define SLAM_CLOCK_HPP

#include <sys/time.h>

namespace slam
{
	class Clock
	{
	public:
	
		virtual timeval now()
		{
			timeval tv;
			gettimeofday(&tv, NULL);
			return tv;
		}
	};
}

#endif