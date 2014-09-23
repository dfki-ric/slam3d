#ifndef SLAM3D_POSEGRAPH_HPP
#define SLAM3D_POSEGRAPH_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace slam3d
{
	typedef pcl::PointXYZI PointType;
	typedef pcl::PointCloud<PointType> PointCloud;
	
	class Node
	{
	public:
		Node();
		~Node();
		
	private:
		PointCloud* mPointCloud;
		
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