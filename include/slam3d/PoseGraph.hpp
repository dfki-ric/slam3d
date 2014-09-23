#ifndef SLAM3D_POSEGRAPH_HPP
#define SLAM3D_POSEGRAPH_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace slam3d
{	
	class Node
	{
	public:
		Node();
		~Node();
		
	private:
		pcl::PointCloud<pcl::PointXYZ>* mPointCloud;
		
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
	};
}

#endif