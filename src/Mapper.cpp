#include <slam3d/Mapper.hpp>

#include <pcl/filters/voxel_grid.h>

#include <string.h>

using namespace slam3d;

Mapper::Mapper()
{

}

Mapper::~Mapper()
{

}

std::string Mapper::getStatusMessage()
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
	
	Node newNode;
	if(mPoseGraph.getNodeCount() == 0)
	{
		newNode.setPointCloud(filtered_scan);
	}else
	{
		mICP.setInputSource(filtered_scan);
		mICP.setInputTarget(mPoseGraph.getLastNode().getPointCloud());
		
		PointCloud* icp_result = new PointCloud();
		PointCloud::ConstPtr icp_result_ptr(icp_result);
		mICP.align(*icp_result);
		
		mStatusMessage.str(std::string());
		mStatusMessage << "Converged: " << mICP.hasConverged() << " / score: " << mICP.getFitnessScore() << std::endl;
		ICP::Matrix4 transform = mICP.getFinalTransformation();
		newNode.setPointCloud(icp_result_ptr);
	}

	mPoseGraph.addNode(newNode);
}

PointCloud::ConstPtr Mapper::getLastScan()
{
	return mPoseGraph.getLastNode().getPointCloud();
}
