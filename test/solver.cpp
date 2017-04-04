#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE SolverTest

#include <G2oSolver.hpp>
#include <FileLogger.hpp>

#include <iostream>
#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_CASE(optimization)
{
	slam3d::Clock clock;
	slam3d::FileLogger logger(clock, "solver.log");
	slam3d::Solver* solver = new slam3d::G2oSolver(&logger);
	
	slam3d::Transform pose(Eigen::Translation<double, 3>(0,0,0));

	slam3d::Transform tf_1_2(Eigen::Translation<double, 3>(1,0,0));
	slam3d::Transform tf_2_3(Eigen::Translation<double, 3>(0,1,0));
	slam3d::Transform tf_3_1(Eigen::Translation<double, 3>(-0.8, -0.7, 0.1));
	slam3d::Covariance cov = slam3d::Covariance::Identity();
	
	solver->addNode(1, pose);
	solver->addNode(2, pose);
	solver->addNode(3, pose);
	BOOST_CHECK_THROW(solver->addNode(3, pose), slam3d::Solver::DuplicateVertex);

	solver->addConstraint(1,2,tf_1_2);
	solver->addConstraint(2,3,tf_2_3);
	solver->addConstraint(3,1,tf_3_1, cov);
	BOOST_CHECK_THROW(solver->addConstraint(1,4,tf_1_2), slam3d::Solver::BadEdge);
	BOOST_CHECK_THROW(solver->setFixed(4), slam3d::Solver::UnknownVertex);

	solver->saveGraph("graph_original.g2o");
	solver->setFixed(1);

	BOOST_CHECK(solver->compute());
	slam3d::IdPoseVector corr = solver->getCorrections();
	BOOST_CHECK_EQUAL(corr.size(), 3);

	solver->addNode(4, pose);
	slam3d::Transform tf_3_4(Eigen::Translation<double, 3>(0, 1, 0));
	solver->addConstraint(3,4,tf_3_4, cov);

	BOOST_CHECK(solver->compute());
	corr = solver->getCorrections();
	BOOST_CHECK_EQUAL(corr.size(), 4);

	solver->saveGraph("graph_optimized.g2o");
}
