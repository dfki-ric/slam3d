#ifndef SLAM_MEASUREMENT_H
#define SLAM_MEASUREMENT_H

namespace slam
{
	class Measurement
	{
	public:
		Measurement(unsigned int i){ id = i;}
	private:
		unsigned int id;
	};
}

#endif