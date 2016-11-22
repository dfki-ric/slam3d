#ifndef SLAM_G2O_SOLVER_HPP
#define SLAM_G2O_SOLVER_HPP

#include "Solver.hpp"
#include <g2o/core/sparse_optimizer.h>

namespace slam3d
{	
	/**
	 * @class G2oSolver
	 * @brief A solver for graph otimization that uses the g2o-backend.
	 * @details See: https://github.com/RainerKuemmerle/g2o for documentation
	 * on the backend.
	 */
	class G2oSolver : public Solver
	{
	public:
		G2oSolver(Logger* logger);
		~G2oSolver();
		
		void addNode(unsigned id, Transform pose);
		void addConstraint(unsigned source, unsigned target, Transform tf, Covariance cov);
		void setFixed(unsigned id);
		bool compute();
		void clear();
		void saveGraph(std::string filename);
		
		IdPoseVector getCorrections();
		
	protected:
		g2o::SparseOptimizer mOptimizer;
		g2o::HyperGraph::VertexSet mNewVertices;
		g2o::HyperGraph::EdgeSet mNewEdges;
		
		IdPoseVector mCorrections;
		bool mInitialized;
	};
}

#endif
