#include <slam3d/Mapper.hpp>

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
	Node newNode;
	if(mPoseGraph.getNodeCount() == 0)
	{
		newNode.setPointCloud(scan);
	}else
	{
		mICP.setInputSource(scan);
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
