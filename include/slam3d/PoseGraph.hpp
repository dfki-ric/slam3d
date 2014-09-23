#ifndef SLAM3D_POSEGRAPH_HPP
#define SLAM3D_POSEGRAPH_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace slam3d
{
	typedef pcl::PointXYZI PointType;
	typedef pcl::PointCloud<PointType> PointCloud;
	
	class Node
	{
	public:
		Node();
		~Node();
		
		void setPointCloud(PointCloud::ConstPtr pcl) { mPointCloud = pcl; }
		PointCloud::ConstPtr getPointCloud() { return mPointCloud; }
		
	private:
		PointCloud::ConstPtr mPointCloud;
		
	};
	
	class Edge
	{
	public:
		Edge();
		~Edge();
		
	private:
		Node* mSource;
		Node* mTarget;
	};
	
	class PoseGraph
	{
	public:
		PoseGraph();
		~PoseGraph();
		
		unsigned int getNodeCount() { return mNodeList.size(); }
		unsigned int getEdgeCount() { return mEdgeList.size(); }
		
		void addNode(Node& n);
		void addEdge(Node& source, Node& target);
		Node getLastNode();
		
	private:
		std::vector<Node> mNodeList;
		std::vector<Edge> mEdgeList;
	};
}

#endif