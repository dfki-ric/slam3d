#ifndef SLAM_SOLVER_HPP
#define SLAM_SOLVER_HPP

#include "PoseGraph.hpp"

#include <vector>

namespace slam
{
	typedef std::pair<int, Transform> IdPose;
	typedef std::vector<IdPose> IdPoseVector;
	
	class Solver
	{
	public:
		virtual void addNode(const VertexObject &v, int id) = 0;
		virtual void addConstraint(const EdgeObject &e, int source, int target) = 0;
		virtual void compute() = 0;
		
		virtual IdPoseVector getCorrections() = 0;
	};
}

#endif