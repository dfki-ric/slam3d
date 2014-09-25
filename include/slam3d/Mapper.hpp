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

		/**
		 * @brief Construct a single Pointcloud from all scans using their corrected poses.
		 * @return Accumulated PointCloud from all scans
		 */
		PointCloud::Ptr getAccumulatedCloud() const;
		
		Pose getCurrentPose() { return mCurrentPose; }
		
	private:
		PoseGraph mPoseGraph;
		
		ICP mICP;
		Pose mCurrentPose;
		
		std::stringstream mStatusMessage;
	};
}

#endif