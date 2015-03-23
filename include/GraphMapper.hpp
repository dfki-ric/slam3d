#ifndef SLAM_GRAPHMAPPER_HPP
#define SLAM_GRAPHMAPPER_HPP

#include "PoseGraph.hpp"
#include "Solver.hpp"
#include "Odometry.hpp"

namespace slam
{
	class GraphMapper
	{
	public:
		GraphMapper(Logger* log);
		~GraphMapper();

		void setSolver(Solver* solver);
		bool optimize();
		void addReading(Measurement* m);

	private:
		PoseGraph mPoseGraph;
		Solver* mSolver;
		Logger* mLogger;
		Odometry* mOdometry;
	};
}

#endif