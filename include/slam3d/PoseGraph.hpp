#ifndef SLAM3D_POSEGRAPH_HPP
#define SLAM3D_POSEGRAPH_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace slam3d
{
	// Type definitions of various components
	typedef float ScalarType;
	typedef pcl::PointXYZI PointType;
	typedef pcl::PointCloud<PointType> PointCloud;
	typedef Eigen::Matrix<ScalarType, 4, 4> Pose;
	
	/**
	 * @class Node
	 * @author Sebastian Kasperski
	 * @date 09/25/14
	 * @file PoseGraph.hpp
	 * @brief Represents a node in the PoseGraph, containing a PointCloud
	 * and the odometric/corrected Pose where it has been collected.
	 */
	class Node
	{
	public:
		Node();
		~Node();
		
		void setPointCloud(PointCloud::ConstPtr pcl) { mPointCloud = pcl; }
		PointCloud::ConstPtr getPointCloud() { return mPointCloud; }
		
	private:
		PointCloud::ConstPtr mPointCloud;
		
		Pose mOdometricPose;
		Pose mCorrectedPose;
	};
	
	/**
	 * @class Edge
	 * @author Sebastian Kasperski
	 * @date 09/25/14
	 * @file PoseGraph.hpp
	 * @brief 
	 */
	class Edge
	{
	public:
		Edge();
		~Edge();
		
	private:
		Node* mSource;
		Node* mTarget;
	};
	
	/**
	 * @class PoseGraph
	 * @author Sebastian Kasperski
	 * @date 09/25/14
	 * @file PoseGraph.hpp
	 * @brief 
	 */
	class PoseGraph
	{
	public:
		PoseGraph();
		~PoseGraph();
		
		unsigned int getNodeCount() { return mNodeList.size(); }
		unsigned int getEdgeCount() { return mEdgeList.size(); }
		
		void addNode(Node& n);
		void addEdge(Node& source, Node& target);
		Node getLastNode() const;
		
	private:
		std::vector<Node> mNodeList;
		std::vector<Edge> mEdgeList;
	};
}

#endif