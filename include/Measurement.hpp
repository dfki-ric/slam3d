#ifndef SLAM_MEASUREMENT_HPP
#define SLAM_MEASUREMENT_HPP

#include <Eigen/Geometry>

namespace slam
{
	typedef Eigen::Affine3d Transform;
	typedef Eigen::Matrix<double,6,6> Covariance;
	
	class Measurement
	{
	public:
		Measurement(unsigned int i){ id = i;}
	private:
		unsigned int id;
	};
}

#endif