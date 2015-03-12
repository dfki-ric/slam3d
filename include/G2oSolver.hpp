#ifndef SLAM_G2O_SOLVER_HPP
#define SLAM_G2O_SOLVER_HPP

#include "Solver.hpp"
#include "g2o/core/sparse_optimizer.h"

#include <exception>

namespace slam
{	
	class G2oSolver : public Solver
	{
	public:
		G2oSolver(Logger* logger);
		~G2oSolver();
		
		void addNode(const VertexObject &v, int id);
		void addConstraint(const EdgeObject &e, int source, int target);
		void compute();
		
		IdPoseVector getCorrections();
		
	protected:
		g2o::SparseOptimizer mOptimizer;
		IdPoseVector mCorrections;
	};
}

#endif