#ifndef SLAM_GRAPHMAPPER_HPP
#define SLAM_GRAPHMAPPER_HPP

#include "Odometry.hpp"
#include "Sensor.hpp"

#include <graph_analysis/BaseGraph.hpp>
#include <flann/flann.hpp>
#include <map>

namespace slam
{
	/**
	 * @class VertexObject
	 * @author Sebastian Kasperski
	 * @date 03/16/15
	 * @file GraphMapper.hpp
	 * @brief Object attached to a vertex in the pose graph.
	 * Contains a pointer to an abstract measurement, which could
	 * be anything, e.g. a range scan, point cloud or image.
	 */
	class VertexObject : public graph_analysis::Vertex
	{
	public:
		typedef boost::shared_ptr<VertexObject> Ptr;

		Transform odometric_pose;
		Transform corrected_pose;
		Measurement* measurement;

		std::string getClassName() const { return "slam::VertexObject"; }
		std::string toString() const { return "VertexObject-instance"; }
	};
	
	/**
	 * @class EdgeObject
	 * @author Sebastian Kasperski
	 * @date 03/16/15
	 * @file GraphMapper.hpp
	 * @brief Object attached to an edge in the pose graph.
	 * Contains the relative transform from source to target.
	 */
	class EdgeObject : public graph_analysis::Edge
	{
	public:
		typedef boost::shared_ptr<EdgeObject> Ptr;

		Transform transform;
		Covariance covariance;
		unsigned int edge_type;

		std::string getClassName() const { return "slam::EdgeObject"; }
		std::string toString() const { return "EdgeObject-instance"; }
	};

	typedef std::vector<VertexObject::Ptr> VertexList;
	typedef std::vector<EdgeObject::Ptr> EdgeList;
	typedef std::map<std::string, Sensor*> SensorList;
	typedef std::map<std::string, VertexObject::Ptr> LastVertexMap;
	
	typedef flann::Index< flann::L2<float> > NeighborIndex;
	
	class Solver;
	
	/**
	 * @class GraphMapper
	 * @author Sebastian Kasperski
	 * @date 04/27/15
	 * @file GraphMapper.hpp
	 * @brief 
	 */
	class GraphMapper
	{
	public:
		GraphMapper(Logger* log);
		~GraphMapper();

		void setSolver(Solver* solver);
		void setOdometry(Odometry* odom);
		void registerSensor(Sensor* s);
		bool optimize();

		void addReading(Measurement* m);
		Transform getCurrentPose() { return mCurrentPose; }
		
		VertexList getVerticesFromSensor(const std::string& sensor);
		EdgeList getEdgesFromSensor(const std::string& sensor);
		
		void buildNeighborIndex();
		VertexList getNearbyVertices(VertexObject::Ptr vertex, float radius);

	private:
		graph_analysis::BaseGraph::Ptr mPoseGraph;
		VertexObject::Ptr mLastVertex;
		LastVertexMap mLastVertices;

		Solver* mSolver;
		Logger* mLogger;
		Odometry* mOdometry;
		SensorList mSensors;
		
		Transform mCurrentPose;
		
		// Index to use nearest neighbor search
		flann::SearchParams mSearchParams;
		NeighborIndex mIndex;
		std::map<int, VertexObject::Ptr> mIndexMap;
	};
}

#endif
