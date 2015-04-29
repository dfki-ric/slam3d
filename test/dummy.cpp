#include <G2oSolver.hpp>

#include <iostream>

int main()
{
	slam::Clock clock;
	slam::Logger logger(clock);
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
	
	solver->addConstraint(1,2,tf_1_2);
	solver->addConstraint(2,3,tf_2_3);
	solver->addConstraint(3,1,tf_3_1, cov);
	
	solver->saveGraph("dummy1.g2o");
	solver->setFixed(1);
	solver->compute();
	solver->saveGraph("dummy2.g2o");
	slam::IdPoseVector corr = solver->getCorrections();
	std::cout << "Results:" << std::endl;
	for(slam::IdPoseVector::iterator c = corr.begin(); c < corr.end(); c++)
	{
		std::cout << "Vertex " << c->first << ": Correction = ("
		<< c->second.translation()[0] << ","
		<< c->second.translation()[1] << ","
		<< c->second.translation()[2] << ")"<< std::endl;
	}
	return 0;
}