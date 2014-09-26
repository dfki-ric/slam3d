#include <slam3d/PoseGraph.hpp>

using namespace slam3d;

Node::Node()
{
	mOdometricPose = Pose::Identity();
	mCorrectedPose = Pose::Identity();
}

Node::Node(PointCloud::ConstPtr pcl, Pose p)
{
	mPointCloud = pcl;
	mOdometricPose = p;
	mCorrectedPose = p;
}

Node::~Node()
{
	
}

Edge::Edge()
{
	
}

Edge::~Edge()
{
	
}

PoseGraph::PoseGraph()
{
	
}

PoseGraph::~PoseGraph()
{
	
}

void PoseGraph::addNode(Node& n)
{
	mNodeList.push_back(n);
}

Node PoseGraph::getLastNode() const
{
	if(mNodeList.empty())
	{
		Node n;
		return n;
	}else
	{
		return mNodeList.back();
	}
}