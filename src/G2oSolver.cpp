#include "G2oSolver.hpp"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include "boost/format.hpp"

using namespace slam;

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

G2oSolver::G2oSolver(Logger* logger) : Solver(logger)
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
	// Check that given id has not been added before
	if(mOptimizer.vertex(id) != NULL)
	{
		throw DuplicateVertex(id);
	}
	
	// Set current pose and id
	g2o::VertexSE3* poseVertex = new g2o::VertexSE3;
	poseVertex->setEstimate(vertex.corrected_pose);  //Eigen::Isometry3d
	poseVertex->setId(id);
	
	// Add the vertex to the optimizer
	mOptimizer.addVertex(poseVertex);
}

void G2oSolver::addConstraint(const EdgeObject &edge, int source, int target)
{
	// Create a new edge
	g2o::EdgeSE3* constraint = new g2o::EdgeSE3();
	
	// Set source and target
	constraint->vertices()[0] = mOptimizer.vertex(source);
	constraint->vertices()[1] = mOptimizer.vertex(target);
	if(constraint->vertices()[0] == NULL || constraint->vertices()[1] == NULL)
	{
		delete constraint;
		throw BadEdge(source, target);
	}
	
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
		mLogger->message(ERROR, "No Node with ID 0 found!");
		return;
	}
	first->setFixed(true);

	// Do the graph optimization
	mOptimizer.initializeOptimization();
	mOptimizer.computeActiveErrors();
	int iter = mOptimizer.optimize(500);
	if (iter <= 0)
	{		
		mLogger->message(ERROR, "Optimization failed!");
		return;
	}

//	std::cout << "Optimization finished after " << iter << " iterations." << std::endl;
	
	mLogger->message(INFO ,(boost::format("Optimization finished after %1% iterations.") % iter).str());

	// Clear previous optimization result
	mCorrections.clear();

	// Write the result so it can be used by the mapper
	g2o::SparseOptimizer::VertexContainer nodes = mOptimizer.activeVertices();
	for (g2o::SparseOptimizer::VertexContainer::const_iterator n = nodes.begin(); n < nodes.end(); n++)
	{
		g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(*n);
		assert(vertex);
		Eigen::Isometry3d iso = vertex->estimate();
		mCorrections.push_back(IdPose((*n)->id(), iso));
	}
}

IdPoseVector G2oSolver::getCorrections()
{
	return mCorrections;
}