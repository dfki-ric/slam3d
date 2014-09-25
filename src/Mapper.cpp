#include <slam3d/Mapper.hpp>

#include <pcl/filters/voxel_grid.h>

#include <string.h>
#include <iostream>

using namespace slam3d;

Mapper::Mapper()
{
	mCurrentPosition = ICP::Matrix4::Identity();
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
	
	PointCloud::Ptr filtered_scan_2(new PointCloud);
	filtered_scan_2->header = filtered_scan->header;
//	double min = 1000;
//	double max = -1000;
	for(PointCloud::iterator p = filtered_scan->begin(); p < filtered_scan->end(); p++)
	{
//		if(p->z < min) min = p->z;
//		if(p->z > max) max = p->z;
		if(p->z > 1)
		{
			filtered_scan_2->push_back(*p);
		}
	}
//	std::cout << "Height-Range: " << min << " .. " << max << std::endl;
	
	Node newNode;
	newNode.setPointCloud(filtered_scan_2);
	
	if(mPoseGraph.getNodeCount() > 0)
	{
		mICP.setInputSource(filtered_scan_2);
		mICP.setInputTarget(mPoseGraph.getLastNode().getPointCloud());
		
		PointCloud* icp_result = new PointCloud();
		PointCloud::ConstPtr icp_result_ptr(icp_result);
		mICP.align(*icp_result);
		
		mStatusMessage.str(std::string());
		mStatusMessage << "Converged: " << mICP.hasConverged() << " / score: " << mICP.getFitnessScore() << std::endl;
		
		// Get position of the new scan
		mCurrentPosition = mICP.getFinalTransformation() * mCurrentPosition;
	}

	mPoseGraph.addNode(newNode);
}

PointCloud::ConstPtr Mapper::getLastScan()
{
	return mPoseGraph.getLastNode().getPointCloud();
}
