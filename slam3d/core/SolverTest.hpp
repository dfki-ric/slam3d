#include <slam3d/core/Solver.hpp>

#include <iostream>
#include <boost/test/unit_test.hpp>

using namespace slam3d;

void test_optimization(Solver* solver)
{	
	Transform pose(Eigen::Translation<double, 3>(0,0,0));

	TransformWithCovariance tf_1_2(Transform(Eigen::Translation<double, 3>(1,0,0)), Covariance<6>::Identity());
	TransformWithCovariance tf_2_3(Transform(Eigen::Translation<double, 3>(0,1,0)), Covariance<6>::Identity());
	TransformWithCovariance tf_3_1(Transform(Eigen::Translation<double, 3>(-0.8, -0.7, 0.1)), Covariance<6>::Identity());
	TransformWithCovariance tf_3_4(Transform(Eigen::Translation<double, 3>(0, 1, 0)), Covariance<6>::Identity());
	
	solver->addVertex(1, pose);
	solver->addVertex(2, pose);
	solver->addVertex(3, pose);
	BOOST_CHECK_THROW(solver->addVertex(3, pose), Solver::DuplicateVertex);

	SE3Constraint::Ptr c_1_2(new SE3Constraint("DummySensor", tf_1_2));
	SE3Constraint::Ptr c_2_3(new SE3Constraint("DummySensor", tf_2_3));
	SE3Constraint::Ptr c_3_1(new SE3Constraint("DummySensor", tf_3_1));
	SE3Constraint::Ptr c_3_4(new SE3Constraint("DummySensor", tf_3_4));

	solver->addEdge(1,2,c_1_2);
	solver->addEdge(2,3,c_2_3);
	solver->addEdge(3,1,c_3_1);

	BOOST_CHECK_THROW(solver->addEdge(1,4,c_1_2), Solver::BadEdge);
	BOOST_CHECK_THROW(solver->setFixed(4), Solver::UnknownVertex);

	BOOST_CHECK_NO_THROW(solver->saveGraph("graph_original.g2o"));
	BOOST_CHECK_NO_THROW(solver->setFixed(1));

	solver->compute();
	IdPoseVector corr = solver->getCorrections();
	BOOST_CHECK_EQUAL(corr.size(), 3);

	solver->addVertex(4, pose);	
	solver->addEdge(3,4,c_3_4);

	solver->compute();
	corr = solver->getCorrections();
	BOOST_CHECK_EQUAL(corr.size(), 4);

	BOOST_CHECK_NO_THROW(solver->saveGraph("graph_optimized.g2o"));
}
