#include "LaserOdometry.hpp"

#include <map>
#include <utility> 

// [LOAM] Zhang, J., & Singh, S. (n.d.). LOAM : Lidar Odometry and Mapping in Real-time.

using namespace slam3d;

LaserOdometry::LaserOdometry()
{
	
}

LaserOdometry::~LaserOdometry()
{
	
}

void LaserOdometry::addScan(PointCloud::ConstPtr scan, double timestamp)
{
	unsigned int cloudSize = scan->points.size();

	// Calculate c-Values [LOAM: V-A (1)]
	std::vector< std::pair<unsigned int, double> > c_values[4];
	unsigned int sectionSize = (cloudSize - 10) / 4.0;
	unsigned int i = 5;
	for(unsigned int section = 0; section < 4; section++)
	{
		for(unsigned int c = 0; c < sectionSize; c++, i++)
		{
			// TODO: Multiply with defined kernel instead of this...
			double diffX = scan->points[i - 5].x + scan->points[i - 4].x 
				+ scan->points[i - 3].x + scan->points[i - 2].x 
				+ scan->points[i - 1].x - 10 * scan->points[i].x 
				+ scan->points[i + 1].x + scan->points[i + 2].x
				+ scan->points[i + 3].x + scan->points[i + 4].x
				+ scan->points[i + 5].x;
			double diffY = scan->points[i - 5].y + scan->points[i - 4].y 
				+ scan->points[i - 3].y + scan->points[i - 2].y 
				+ scan->points[i - 1].y - 10 * scan->points[i].y 
				+ scan->points[i + 1].y + scan->points[i + 2].y
				+ scan->points[i + 3].y + scan->points[i + 4].y
				+ scan->points[i + 5].y;
			double diffZ = scan->points[i - 5].z + scan->points[i - 4].z 
				+ scan->points[i - 3].z + scan->points[i - 2].z 
				+ scan->points[i - 1].z - 10 * scan->points[i].z 
				+ scan->points[i + 1].z + scan->points[i + 2].z
				+ scan->points[i + 3].z + scan->points[i + 4].z
				+ scan->points[i + 5].z;

			c_values[section].push_back(std::make_pair(i, diffX * diffX + diffY * diffY + diffZ * diffZ));
		}
	}
	
	
}

void LaserOdometry::finishSweep()
{
	
}