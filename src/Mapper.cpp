#include <slam3d/Mapper.hpp>

#include <pcl/filters/voxel_grid.h>

#include <string.h>

using namespace slam3d;

Mapper::Mapper()
{
	mCurrentPose = Pose::Identity();
}

Mapper::~Mapper()
{

}

std::string Mapper::getStatusMessage() const
{
	return mStatusMessage.str();
}

void Mapper::addScan(PointCloud::ConstPtr scan)
{
	// Downsample the scan
	PointCloud::Ptr filtered_scan(new PointCloud);
	pcl::VoxelGrid<PointType> grid;
	grid.setLeafSize (0.25, 0.25, 0.25);
	grid.setInputCloud(scan);
	grid.filter(*filtered_scan);
	
	PointCloud::Ptr filtered_scan_2(new PointCloud);
	filtered_scan_2->header = filtered_scan->header;

	for(PointCloud::iterator p = filtered_scan->begin(); p < filtered_scan->end(); p++)
	{
		if(p->z > 1)
			filtered_scan_2->push_back(*p);
	}
	
	Node newNode;
	newNode.setPointCloud(filtered_scan_2);
	
	if(mPoseGraph.getNodeCount() > 0)
	{
		mICP.setInputSource(filtered_scan_2);
		mICP.setInputTarget(mPoseGraph.getLastNode().getPointCloud());
		
		PointCloud icp_result;
		mICP.align(icp_result);
		
		mStatusMessage.str(std::string());
		mStatusMessage << "Converged: " << mICP.hasConverged() << " / score: " << mICP.getFitnessScore() << std::endl;
		
		// Get position of the new scan
		mCurrentPose = mICP.getFinalTransformation() * mCurrentPose;
	}

	mPoseGraph.addNode(newNode);
}

PointCloud::Ptr Mapper::getLastScan() const
{
	PointCloud::Ptr pc(new PointCloud(*mPoseGraph.getLastNode().getPointCloud()));
	return pc;
}

PointCloud::Ptr Mapper::getAccumulatedCloud() const
{
	PointCloud::Ptr accumulatedCloud(new PointCloud);
	return accumulatedCloud;
}
