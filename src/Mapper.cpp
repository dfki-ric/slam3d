#include <slam3d/Mapper.hpp>

#include <pcl/filters/voxel_grid.h>

#include <string.h>

#define FLT_SIZE 0.025

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

void Mapper::addScan(PointCloud::ConstPtr scan, Pose pose)
{
	// Downsample the scan
	PointCloud::Ptr filtered_scan(new PointCloud);
	pcl::VoxelGrid<PointType> grid;
	grid.setLeafSize (FLT_SIZE, FLT_SIZE, FLT_SIZE);
	grid.setInputCloud(scan);
	grid.filter(*filtered_scan);
	
	Node newNode(filtered_scan, pose);

/*	
	// Remove ground plane (hack)
	PointCloud::Ptr filtered_scan_2(new PointCloud);
	filtered_scan_2->header = filtered_scan->header;

	for(PointCloud::iterator p = filtered_scan->begin(); p < filtered_scan->end(); p++)
	{
		if(p->z > -1)
			filtered_scan_2->push_back(*p);
	}
	
	Node newNode(filtered_scan_2, pose);
*/	
	if(mPoseGraph.getNodeCount() > 0)
	{
		// Set corrected pose based on last node's correction
		Node lastNode = mPoseGraph.getLastNode();
		Pose diff = lastNode.getCorrectedPose() * lastNode.getOdometricPose().inverse();
		newNode.setCorrectedPose(pose * diff);
		
		mICP.setInputSource(filtered_scan);
		mICP.setInputTarget(mPoseGraph.getLastNode().getPointCloud());
		
//		mICP.setMaxCorrespondenceDistance(0.5);
//		mICP.setTransformationEpsilon (1e-6);

		// Calculate odometry difference between last scan and this one
		Pose odom = lastNode.getOdometricPose().inverse() * pose;

		PointCloud icp_result;
		mICP.align(icp_result, odom);
		
		mStatusMessage.str(std::string());
		mStatusMessage << "Converged: " << mICP.hasConverged() << " / score: " << mICP.getFitnessScore() << std::endl;
		
		// Get position of the new scan
		mCurrentPose = mCurrentPose * mICP.getFinalTransformation();
		newNode.setCorrectedPose(mCurrentPose);
	}

	mPoseGraph.addNode(newNode);
	createAccumulatedCloud();
}

PointCloud::Ptr Mapper::getLastScan() const
{
	PointCloud::Ptr pc(new PointCloud(*mPoseGraph.getLastNode().getPointCloud()));
	return pc;
}

void Mapper::createAccumulatedCloud()
{
	PointCloud::Ptr accumulatedCloud(new PointCloud);
	NodeList allNodes = mPoseGraph.getAllNodes();
	int count = 0;
	for(NodeList::reverse_iterator n = allNodes.rbegin(); n != allNodes.rend(); n++)
	{
		Pose p = n->getCorrectedPose();
		PointCloud::ConstPtr pc = n->getPointCloud();
		
		PointCloud pc_tf;
		pcl::transformPointCloud(*pc, pc_tf, p);
		*accumulatedCloud += pc_tf;
		count++;
		if(count > 10) break;
	}
	accumulatedCloud->header.frame_id = "map";
	accumulatedCloud->header.stamp = mPoseGraph.getLastNode().getPointCloud()->header.stamp;
	
	// Downsample the result
	PointCloud::Ptr filtered_cloud(new PointCloud);
	pcl::VoxelGrid<PointType> grid;
	
	grid.setLeafSize (FLT_SIZE, FLT_SIZE, FLT_SIZE);
	grid.setInputCloud(accumulatedCloud);
	grid.filter(*filtered_cloud);

	mAccumulatedCloud = filtered_cloud;
}

