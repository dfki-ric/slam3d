#ifndef SLAM_GRAPHMAPPER_HPP
#define SLAM_GRAPHMAPPER_HPP

#include "PoseGraph.hpp"
#include "Solver.hpp"

namespace slam
{
	class GraphMapper
	{
	public:
		GraphMapper();
		~GraphMapper();
		
		void setSolver(Solver* solver);
		
	protected:
		
		
	protected:
		PoseGraph mPoseGraph;
		Solver* mSolver;
	};
}

#endif