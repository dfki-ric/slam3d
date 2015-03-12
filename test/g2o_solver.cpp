#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN

#define BOOST_TEST_MODULE G2O_SOLVER

#include <boost/test/unit_test.hpp>
#include <G2oSolver.hpp>

#include <iostream>


BOOST_AUTO_TEST_CASE(g2o_solver_1)
{
	slam::G2oSolver solver;
	slam::VertexObject v1, v2, v3;
	slam::EdgeObject e1,e2, e3;
	solver.addNode(v1, 1);
	solver.addNode(v2, 2);
	solver.addNode(v3, 3);
	
	solver.addConstraint(e1, 1,2);
	solver.addConstraint(e2, 2,3);
	BOOST_CHECK_THROW(solver.addConstraint(e3, 3,4), slam::BadEdge);
}