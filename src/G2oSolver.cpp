#include "G2oSolver.hpp"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

using namespace slam;

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

G2oSolver::G2oSolver()
{
	// Initialize the SparseOptimizer
	SlamLinearSolver* linearSolver = new SlamLinearSolver();
	linearSolver->setBlockOrdering(false);
	SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
	mOptimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(blockSolver));
}

G2oSolver::~G2oSolver()
{
	// Destroy all the singletons
	g2o::Factory::destroy();
	g2o::OptimizationAlgorithmFactory::destroy();
	g2o::HyperGraphActionLibrary::destroy();
}

void G2oSolver::addNode(const VertexObject &vertex, int id)
{
	g2o::VertexSE3* poseVertex = new g2o::VertexSE3;
	Eigen::Isometry3d isometry = vertex.corrected_pose;
	poseVertex->setEstimate(isometry);
	poseVertex->setId(id);
	mOptimizer.addVertex(poseVertex);
}

void G2oSolver::addConstraint(const EdgeObject &edge, int source, int target)
{
	// Create a new edge
	g2o::EdgeSE3* constraint = new g2o::EdgeSE3();
	
	// Set source and target
	constraint->vertices()[0] = mOptimizer.vertex(source);
	constraint->vertices()[1] = mOptimizer.vertex(target);
	
	// Set the measurement (odometry distance between vertices)
	constraint->setMeasurement(edge.transform);   // Eigen::Isometry3d
	constraint->setInformation(edge.covariance);  // Eigen::Matrix<double,6,6>
	
	// Add the constraint to the optimizer
	mOptimizer.addEdge(constraint);
}

void G2oSolver::compute()
{
	// Fix the first node in the graph to hold the map in place
	g2o::OptimizableGraph::Vertex* first = mOptimizer.vertex(0);
	if(!first)
	{
//		ROS_ERROR("[g2o] No Node with ID 0 found!");
		return;
	}
	first->setFixed(true);

	// Do the graph optimization
	mOptimizer.initializeOptimization();
	mOptimizer.computeActiveErrors();
	int iter = mOptimizer.optimize(500);
	if (iter > 0)
	{
//		ROS_INFO("[g2o] Optimization finished after %d iterations.", iter);
	}else
	{
//		ROS_ERROR("[g2o] Optimization failed, result might be invalid!");
		return;
	}

	// Clear previous optimization result
	mCorrections.clear();

	// Write the result so it can be used by the mapper
	g2o::SparseOptimizer::VertexContainer nodes = mOptimizer.activeVertices();
	for (g2o::SparseOptimizer::VertexContainer::const_iterator n = nodes.begin(); n != nodes.end(); n++)
	{
		g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(*n);
		if(vertex)
		{
			Eigen::Isometry3d iso = vertex->estimate();
			mCorrections.push_back(IdPose((*n)->id(), iso));
		}else
		{
			// dynamic_cast failed, n did not point to a VertexSE3
			assert(true);
		}
/*
		int dim = (*n)->estimateDimension();
		if(dim > 0)
		{
			double estimate[dim];
			if((*n)->getEstimateData(estimate))
			{
	//			karto::Pose2 pose(estimate[0], estimate[1], estimate[2]);
	//			mCorrections.Add(karto::Pair<int, karto::Pose2>((*n)->id(), pose));
			}else
			{
	//			ROS_ERROR("[g2o] Could not get estimated pose from Optimizer!");
			}
		}
*/	}
}

IdPoseVector G2oSolver::getCorrections()
{
	return mCorrections;
}