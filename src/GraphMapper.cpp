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