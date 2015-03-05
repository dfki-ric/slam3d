#include "G2oSolver.hpp"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/types/icp/types_icp.h"
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

void G2oSolver::addNode()
{
//	karto::Pose2 odom = pVertex->GetVertexObject()->GetCorrectedPose();
	g2o::VertexSE3* poseVertex = new g2o::VertexSE3;
	Eigen::Isometry3d isometry;
	// Fill isometry with data from node
	poseVertex->setEstimate(isometry);
//	poseVertex->setId(pVertex->GetVertexObject()->GetUniqueId());
	mOptimizer.addVertex(poseVertex);
//	ROS_DEBUG("[g2o] Adding node %d.", pVertex->GetVertexObject()->GetUniqueId());
}

void G2oSolver::addConstraint()
{
	g2o::EdgeGICP shift;
	Eigen::Vector3d v0, v1, n0, n1;
	// Fill in data from edge
	shift.pos0 = v0;
	shift.pos1 = v1;
	shift.normal0 = n0;
	shift.normal1 = n1;

	// Build the actual edge
	g2o::Edge_V_V_GICP* edge = new g2o::Edge_V_V_GICP();
	edge->setMeasurement(shift);
	mOptimizer.addEdge(edge);
}

void G2oSolver::compute()
{
	
}

IdPoseVector G2oSolver::getCorrections()
{
	return mCorrections;
}