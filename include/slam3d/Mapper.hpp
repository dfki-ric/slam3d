#ifndef SLAM3D_MAPPER_HPP
#define SLAM3D_MAPPER_HPP

#include <slam3d/PoseGraph.hpp>
#include <pcl/registration/icp.h>
#include <sstream>

namespace slam3d
{
	typedef pcl::IterativeClosestPoint<PointType, PointType> ICP;
	
	class Mapper
	{
	public:
		Mapper();
		~Mapper();
		
		std::string getStatusMessage();
		void addScan(PointCloud::ConstPtr scan);
		PointCloud::ConstPtr getLastScan();
		
	private:
		PoseGraph mPoseGraph;
		
		ICP mICP;
		
		std::stringstream mStatusMessage;
	};
}

#endif