#include <G2oSolver.hpp>

#include <iostream>

int main()
{
	slam::Logger logger;
	slam::G2oSolver* solver = new slam::G2oSolver(&logger);
	slam::VertexObject v1, v2, v3;
	v1.odometric_pose.translation() = Eigen::Vector3d(0,0,0);
	v2.odometric_pose.translation() = Eigen::Vector3d(1.5,0,0);
	v3.odometric_pose.translation() = Eigen::Vector3d(3,0,0);

	slam::EdgeObject e1,e2, e3;
	e1.transform.translation() = Eigen::Vector3d(1,0,0);
	e2.transform.translation() = Eigen::Vector3d(1,0,0);
	solver->addNode(v1, 0);
	solver->addNode(v2, 1);
	solver->addNode(v3, 2);
	
	solver->addConstraint(e1, 0,1);
	solver->addConstraint(e2, 1,2);
	
	solver->compute();
	slam::IdPoseVector corr = solver->getCorrections();
	std::cout << "Results:" << std::endl;
	for(slam::IdPoseVector::iterator c = corr.begin(); c < corr.end(); c++)
	{
		std::cout << "Vertex " << c->first << ": Correction = " << c->second.translation() << std::endl;
	}
	return 0;
}