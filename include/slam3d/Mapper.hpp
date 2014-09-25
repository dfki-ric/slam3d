#ifndef SLAM3D_MAPPER_HPP
#define SLAM3D_MAPPER_HPP

#include <slam3d/PoseGraph.hpp>
#include <pcl/registration/icp.h>

#include <sstream>

namespace slam3d
{
	typedef pcl::IterativeClosestPoint<PointType, PointType, ScalarType> ICP;
	
	class Mapper
	{
	public:
		Mapper();
		~Mapper();
		
		std::string getStatusMessage() const;
		void addScan(PointCloud::ConstPtr scan);
		PointCloud::Ptr getLastScan() const;

		PointCloud::Ptr getAccumulatedCloud() const { return mAccumulatedCloud; }
		
		Pose getCurrentPose() { return mCurrentPose; }
		
	protected:
		void createAccumulatedCloud();
		
	protected:
		PoseGraph mPoseGraph;
		
		ICP mICP;
		Pose mCurrentPose;
		PointCloud::Ptr mAccumulatedCloud;
		
		std::stringstream mStatusMessage;
	};
}

#endif