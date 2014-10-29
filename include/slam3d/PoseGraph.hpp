#ifndef SLAM3D_POSEGRAPH_HPP
#define SLAM3D_POSEGRAPH_HPP

#include "Types.hpp"

#include <vector>

namespace slam3d
{	
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
		Node(PointCloud::ConstPtr pcl, Pose p);
		~Node();
		
		PointCloud::ConstPtr getPointCloud() { return mPointCloud; }
		Pose getOdometricPose() const { return mOdometricPose; }
		Pose getCorrectedPose() const { return mCorrectedPose; }
		
		void setCorrectedPose(Pose p) { mCorrectedPose = p; }

		
	protected:
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
		
	protected:
		Node* mSource;
		Node* mTarget;
	};
	
	typedef std::vector<Node> NodeList;
	typedef std::vector<Edge> EdgeList;
	
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

		NodeList getAllNodes() const { return mNodeList; }
		EdgeList getAllEdges() const { return mEdgeList; }
		
		// Temporary stuff
		Node getLastNode() const;
		
	protected:
		NodeList mNodeList;
		EdgeList mEdgeList;
	};
}

#endif