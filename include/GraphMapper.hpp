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
	
	class GraphMapper
	{
	public:
		GraphMapper(Logger* log);
		~GraphMapper();

		void setSolver(Solver* solver);
		void setOdometry(Odometry* odom);
		bool optimize();
		void addReading(Measurement* m);

	private:
		PoseGraph mPoseGraph;
		Solver* mSolver;
		Logger* mLogger;
		Odometry* mOdometry;
		SensorList mSensors;
	};
}

#endif