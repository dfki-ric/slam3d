#ifndef SLAM_G2O_SOLVER_HPP
#define SLAM_G2O_SOLVER_HPP

#include "Solver.hpp"
#include "g2o/core/sparse_optimizer.h"

namespace slam
{
	class G2oSolver : public Solver
	{
	public:
		G2oSolver();
		~G2oSolver();
		
		void addNode();
		void addConstraint();
		void compute();
		
		IdPoseVector getCorrections();
		
	protected:
		g2o::SparseOptimizer mOptimizer;
		IdPoseVector mCorrections;
	};
}

#endif