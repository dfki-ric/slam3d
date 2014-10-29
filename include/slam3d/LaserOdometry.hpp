#ifndef SLAM3D_LASERODOMETRY_HPP
#define SLAM3D_LASERODOMETRY_HPP

#include "Types.hpp"

namespace slam3d
{
	class LaserOdometry
	{
	public:
		LaserOdometry();
		~LaserOdometry();
		
		void addScan(PointCloud::ConstPtr scan, double timestamp);
		void finishSweep();
	
	protected:
	
		PointCloud mSurfacePoints;
		PointCloud mEdgePoints;
	};
}

#endif