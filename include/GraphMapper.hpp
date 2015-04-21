#ifndef SLAM_GRAPHMAPPER_HPP
#define SLAM_GRAPHMAPPER_HPP

#include "PoseGraph.hpp"
#include "Solver.hpp"
#include "Odometry.hpp"
#include "Sensor.hpp"

#include <graph_analysis/BaseGraph.hpp>
#include <map>

namespace slam
{
	typedef std::map<std::string, Sensor*> SensorList;
	//typedef std::map<std::string, PoseGraph::IdType> LastVertexList;
        typedef std::map<std::string, VertexObject::Ptr> LastVertexMap;
	
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
		
		VertexList getVerticesFromSensor(std::string sensor);
		//EdgeList getEdges(unsigned type = 0) { return mPoseGraph->getEdges(type); }
		//VertexLinkList getVertexLinks(unsigned type = 0){return mPoseGraph.getVertexLinks(type);}

	private:
                graph_analysis::BaseGraph::Ptr mPoseGraph;
                VertexObject::Ptr mLastVertex;
                LastVertexMap mLastVertices;

		Solver* mSolver;
		Logger* mLogger;
		Odometry* mOdometry;
		SensorList mSensors;
		
		Transform mCurrentPose;
	};
}

#endif
