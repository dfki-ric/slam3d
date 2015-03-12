#ifndef SLAM_G2O_SOLVER_HPP
#define SLAM_G2O_SOLVER_HPP

#include "Solver.hpp"
#include "g2o/core/sparse_optimizer.h"

#include <exception>

namespace slam
{
	class BadEdge: public std::exception
	{
	public:
		BadEdge(int s, int t):source(s),target(t){}
		virtual const char* what() const throw()
		{
			char error[100];
			sprintf(error, "Failed to create edge from node %d to %d!", source, target);
			return error;
		}
		
		int source;
		int target;
	};
	
	class G2oSolver : public Solver
	{
	public:
		G2oSolver();
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