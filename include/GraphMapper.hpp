#ifndef SLAM_GRAPHMAPPER_HPP
#define SLAM_GRAPHMAPPER_HPP

#include "PoseGraph.hpp"
#include "Solver.hpp"
#include "Odometry.hpp"
#include "Sensor.hpp"

namespace slam
{
	typedef std::map<std::String, Sensor*> SensorList;
	
	class GraphMapper
	{
	public:
		GraphMapper(Logger* log);
		~GraphMapper();
		
		void registerSensor(std::string& name, Sensor* s);
		void setSolver(Solver* solver);
		bool optimize();
		
		void addReading(Measurement* m);
		
		
	private:
		PoseGraph mPoseGraph;
		Solver* mSolver;
		Logger* mLogger;
		
		SensorList mSensors;
		Odometry* mOdometry;
	};
}

#endif