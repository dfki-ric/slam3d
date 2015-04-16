#ifndef SLAM_GRAPHMAPPER_HPP
#define SLAM_GRAPHMAPPER_HPP

#include "PoseGraph.hpp"
#include "Solver.hpp"
#include "Odometry.hpp"
#include "Sensor.hpp"

#include <map>

namespace slam
{
	typedef std::map<std::string, Sensor*> SensorList;
	typedef std::map<std::string, PoseGraph::IdType> LastVertexList;
	
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

	private:
		PoseGraph mPoseGraph;
		Solver* mSolver;
		Logger* mLogger;
		Odometry* mOdometry;
		SensorList mSensors;
		LastVertexList mLastVertices;
		
		Transform mCurrentPose;
	};
}

#endif