#ifndef SLAM_SOLVER_HPP
#define SLAM_SOLVER_HPP

#include "Measurement.hpp"

#include <vector>

namespace slam
{
	typedef std::pair<int, Transform> IdPose;
	typedef std::vector<IdPose> IdPoseVector;
	
	class Solver
	{
	public:
		virtual void addNode() = 0;
		virtual void addConstraint() = 0;
		virtual void compute() = 0;
		
		virtual IdPoseVector getCorrections() = 0;
	};
}

#endif