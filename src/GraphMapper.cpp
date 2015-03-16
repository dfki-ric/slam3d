#include "GraphMapper.hpp"

using namespace slam;

GraphMapper::GraphMapper()
{
	mSolver = NULL;

}

GraphMapper::~GraphMapper()
{
	

}

void GraphMapper::setSolver(Solver* solver)
{
	mSolver = solver;
}

bool GraphMapper::optimize()
{
	// Give the graph structure to the solver

}