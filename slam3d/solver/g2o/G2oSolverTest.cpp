#define BOOST_TEST_MODULE "G2oSolverTest"

#include "G2oSolver.hpp"

#include <slam3d/core/FileLogger.hpp>
#include <slam3d/core/SolverTest.hpp>

using namespace slam3d;

BOOST_AUTO_TEST_CASE(g2o_optimization)
{
	Clock clock;
	FileLogger logger(clock, "boost_graph.log");
	logger.setLogLevel(DEBUG);
	Solver* solver = new G2oSolver(&logger);
	test_optimization(solver);
	delete solver;
}
