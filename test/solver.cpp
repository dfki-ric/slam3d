#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE SolverTest

#include "G2oSolver.hpp"
#include "FileLogger.hpp"

#include <iostream>
#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_CASE(optimization)
{
	slam::Clock clock;
	slam::FileLogger logger(clock, "g2o.log");
	slam::Solver* solver = new slam::G2oSolver(&logger);
	
	slam::Transform pose_1(Eigen::Translation<double, 3>(0,0,0));
	slam::Transform pose_2(Eigen::Translation<double, 3>(0,0,0));
	slam::Transform pose_3(Eigen::Translation<double, 3>(0,0,0));

	slam::Transform tf_1_2(Eigen::Translation<double, 3>(1,0,0));
	slam::Transform tf_2_3(Eigen::Translation<double, 3>(0,1,0));
	slam::Transform tf_3_1(Eigen::Translation<double, 3>(-0.8, -0.7, 0.1));
	slam::Covariance cov = slam::Covariance::Identity();
	
	solver->addNode(1, pose_1);
	solver->addNode(2, pose_2);
	solver->addNode(3, pose_3);
	BOOST_CHECK_THROW(solver->addNode(3, pose_3), slam::DuplicateVertex);
	
	solver->addConstraint(1,2,tf_1_2);
	solver->addConstraint(2,3,tf_2_3);
	solver->addConstraint(3,1,tf_3_1, cov);
	BOOST_CHECK_THROW(solver->addConstraint(1,4,tf_1_2), slam::BadEdge);
	BOOST_CHECK_THROW(solver->setFixed(4), slam::UnknownVertex);
	
	solver->saveGraph("graph_original.g2o");
	solver->setFixed(1);
	solver->compute();
	solver->saveGraph("graph_optimized.g2o");
	slam::IdPoseVector corr = solver->getCorrections();
	BOOST_CHECK_EQUAL(corr.size(), 3);
}
