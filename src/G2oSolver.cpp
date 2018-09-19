// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2017 S. Kasperski
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "G2oSolver.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

#include "boost/format.hpp"

using namespace slam3d;

typedef g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> SlamLinearSolver;

G2oSolver::G2oSolver(Logger* logger) : Solver(logger)
{
	// Initialize the SparseOptimizer
	SlamLinearSolver* linearSolver = new SlamLinearSolver();
	linearSolver->setBlockOrdering(true);
	g2o::BlockSolver_6_3* blockSolver = new g2o::BlockSolver_6_3(linearSolver);
	mOptimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(blockSolver));
	
	// Set the default terminate action
	g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction;
	mOptimizer.addPostIterationAction(terminateAction);
	
	mInitialized = false;
}

G2oSolver::~G2oSolver()
{
	clear();
}

void G2oSolver::addNode(unsigned id, Transform pose)
{
	// Check that given id has not been added before
	if(mOptimizer.vertex(id) != NULL)
	{
		throw DuplicateVertex(id);
	}
	
	// Set current pose and id
	g2o::VertexSE3* poseVertex = new g2o::VertexSE3;
	poseVertex->setEstimate(pose.cast<double>());  //Eigen::Isometry3d
	poseVertex->setId(id);
	
	// Add the vertex to the optimizer
	mOptimizer.addVertex(poseVertex);
	mNewVertices.insert(poseVertex);
}

void G2oSolver::addConstraint(unsigned source, unsigned target, Transform tf, Covariance cov)
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
	constraint->setMeasurement(tf.cast<double>());            // slam3d::Transform  aka Eigen::Isometry3d
	constraint->setInformation(cov.inverse().cast<double>()); // slam3d::Covariance aka Eigen::Matrix<double,6,6>
	
	// Add the constraint to the optimizer
	mOptimizer.addEdge(constraint);
	mNewEdges.insert(constraint);
}

void G2oSolver::setFixed(unsigned id)
{
	// Fix the node in the graph to hold the map in place
	g2o::OptimizableGraph::Vertex* v = mOptimizer.vertex(id);
	if(!v)
	{
		mLogger->message(ERROR, (boost::format("Could not fix node with ID %1%!") % id).str());
		throw UnknownVertex(id);
	}
	v->setFixed(true);
}

bool G2oSolver::compute()
{
	// need to do something?
	if(mOptimizer.activeVertices().size() == 0 && mNewVertices.size() < 2)
		return true;
	
	// Check input
	if(!mOptimizer.verifyInformationMatrices(true))
	{
		mLogger->message(ERROR, "Failed to verify information matrices!");
		return false;
	}

	// Reset the stop flag that is set by TerminateAction
	bool* stopFlag = mOptimizer.forceStopFlag();
	if(stopFlag)
	{
		*stopFlag = false;
	}
	
	// Do the graph optimization
	if(mInitialized)
	{
		mLogger->message(DEBUG, "Update Initialization.");
		mOptimizer.updateInitialization(mNewVertices, mNewEdges);
	}else
	{
		mLogger->message(DEBUG, "Do first Initialization.");
		mInitialized = mOptimizer.initializeOptimization();
	}
	mNewVertices.clear();
	mNewEdges.clear();
	
	int iter = mOptimizer.optimize(100, false);
	if (iter <= 0)
	{		
		mLogger->message(ERROR, "Optimization failed!");
		return false;
	}
	mLogger->message(INFO ,(boost::format("Optimization finished after %1% iterations.") % iter).str());

	// Clear previous optimization result
	mCorrections.clear();

	// Write the result so it can be used by the mapper
	g2o::SparseOptimizer::VertexContainer nodes = mOptimizer.activeVertices();
	for (g2o::SparseOptimizer::VertexContainer::const_iterator n = nodes.begin(); n < nodes.end(); n++)
	{
		g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(*n);
		assert(vertex);
		Transform iso = Transform(vertex->estimate());
		mCorrections.push_back(IdPose((*n)->id(), iso));
	}
	return true;
}

IdPoseVector G2oSolver::getCorrections()
{
	return mCorrections;
}

void G2oSolver::clear()
{
	mOptimizer.clear();
	mInitialized = false;
}

void G2oSolver::saveGraph(std::string filename)
{
	if(mOptimizer.save(filename.c_str()))
	{
		mLogger->message(INFO, (boost::format("Saved current g2o graph in %1%.") % filename).str());
	}else
	{
		mLogger->message(ERROR, (boost::format("Could not save %1%.") % filename).str());
	}
}
