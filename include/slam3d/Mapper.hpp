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
		
		ICP::Matrix4 getCurrentPosition() { return mCurrentPosition; }
		
	private:
		PoseGraph mPoseGraph;
		
		ICP mICP;
		ICP::Matrix4 mCurrentPosition;
		
		std::stringstream mStatusMessage;
	};
}

#endif