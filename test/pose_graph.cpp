#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE slam3d

#include <boost/test/unit_test.hpp>

#include <PoseGraph.hpp>
#include <G2oSolver.hpp>
#include <FileLogger.hpp>

#include <iostream>

BOOST_AUTO_TEST_CASE(pose_graph_1)
{
	slam::Clock clock;
	slam::FileLogger logger(clock, "pose_graph_1.log");
	
	slam::PoseGraph graph;
	slam::Measurement m1(1, clock.now(), "Sensor");
	slam::Measurement m2(2, clock.now(), "Sensor");
	slam::Measurement m3(3, clock.now(), "Sensor");

	// Create the vertices
	slam::VertexObject vo[3];

	vo[0].measurement = &m1;
	vo[0].odometric_pose = Eigen::Translation<double, 3>(0,0,0);
	vo[0].corrected_pose = Eigen::Translation<double, 3>(0,0,0);
	slam::Vertex v1 = graph.addVertex(vo[0]);

	vo[1].measurement = &m2;
	vo[1].odometric_pose = Eigen::Translation<double, 3>(1,0,0);
	vo[1].corrected_pose = Eigen::Translation<double, 3>(1,0,0);
	slam::Vertex v2 = graph.addVertex(vo[1]);

	vo[2].measurement = &m3;
	vo[2].odometric_pose = Eigen::Translation<double, 3>(1,1,0);
	vo[2].corrected_pose = Eigen::Translation<double, 3>(1,1,0);
	slam::Vertex v3 = graph.addVertex(vo[2]);
	
	// Create the edges
	slam::EdgeObject e_obj;
	e_obj.covariance = slam::Covariance::Identity();
	
	e_obj.transform = Eigen::Translation<double, 3>(1,0,0);
	graph.addEdge(v1, v2, e_obj);
	
	e_obj.transform = Eigen::Translation<double, 3>(0,1,0);
	graph.addEdge(v2, v3, e_obj);
	
	e_obj.transform = Eigen::Translation<double, 3>(-0.8, -0.7, 0.2);
	graph.addEdge(v3, v1, e_obj);
	
	BOOST_CHECK(graph.getMeasurement(v1) == &m1);
	BOOST_CHECK(graph.getMeasurement(v2) == &m2);
	BOOST_CHECK(graph.getMeasurement(v3) == &m3);
	
	BOOST_CHECK(graph.getMeasurement(v1) != &m2);
	BOOST_CHECK(graph.getMeasurement(v1) != &m3);
	
	// Test file output
	std::ofstream file;
	file.open("construction_test.dot");
	graph.dumpGraphViz(file);
	file.close();
	
	// Check before
	for(unsigned int id = 0; id < 3; id++)
	{
		std::cout << "Vertex " << id << ": Before = ("
		<< graph.getVertex(id).corrected_pose.translation()[0] << ","
		<< graph.getVertex(id).corrected_pose.translation()[1] << ","
		<< graph.getVertex(id).corrected_pose.translation()[2] << ")"<< std::endl;
	}
	
	// Optimize the graph
	slam::Solver* solver = new slam::G2oSolver(&logger);
	graph.optimize(solver);
	solver->saveGraph("pose_graph_1_result.g2o");
	
	// Check after
	for(unsigned int id = 0; id < 3; id++)
	{
		std::cout << "Vertex " << id << ": After = ("
		<< graph.getVertex(id).corrected_pose.translation()[0] << ","
		<< graph.getVertex(id).corrected_pose.translation()[1] << ","
		<< graph.getVertex(id).corrected_pose.translation()[2] << ")"<< std::endl;
	}
	
	// Check the result
	slam::IdPoseVector corr = solver->getCorrections();
	std::cout << "Results:" << std::endl;
	for(slam::IdPoseVector::iterator c = corr.begin(); c < corr.end(); c++)
	{
		std::cout << "Vertex " << c->first << ": Correction = ("
		<< c->second.translation()[0] << ","
		<< c->second.translation()[1] << ","
		<< c->second.translation()[2] << ")"<< std::endl;
	}
	
	// Boost-Checks
	for(slam::IdPoseVector::iterator c = corr.begin(); c < corr.end(); c++)
	{
		BOOST_CHECK_EQUAL(graph.getVertex(c->first).corrected_pose.translation()[0], c->second.translation()[0]);
		BOOST_CHECK_EQUAL(graph.getVertex(c->first).corrected_pose.translation()[1], c->second.translation()[1]);
		BOOST_CHECK_EQUAL(graph.getVertex(c->first).corrected_pose.translation()[2], c->second.translation()[2]);
		BOOST_CHECK_EQUAL(graph.getVertex(c->first).odometric_pose.translation()[0], vo[c->first].odometric_pose.translation()[0]);
		BOOST_CHECK_EQUAL(graph.getVertex(c->first).odometric_pose.translation()[1], vo[c->first].odometric_pose.translation()[1]);
		BOOST_CHECK_EQUAL(graph.getVertex(c->first).odometric_pose.translation()[2], vo[c->first].odometric_pose.translation()[2]);
	}
}
/*
BOOST_AUTO_TEST_CASE(g2o_solver_1)
{
	slam::Clock clock;
	slam::Logger logger(clock);
	slam::G2oSolver* solver = new slam::G2oSolver(&logger);
	
	// Create the nodes
	slam::VertexObject v1, v2, v3;
	v1.id = 1;
	v2.id = 2;
	v3.id = 3;
	v1.corrected_pose = Eigen::Translation<double, 3>(0,0,0);
	v2.corrected_pose = Eigen::Translation<double, 3>(0,0,0);
	v3.corrected_pose = Eigen::Translation<double, 3>(0,0,0);
	solver->addNode(v1);
	solver->addNode(v2);
	solver->addNode(v3);
	BOOST_CHECK_THROW(solver->addNode(v3), slam::DuplicateVertex);

	// Create the edges
	slam::EdgeObject e1,e2, e3;
	e1.transform = Eigen::Translation<double, 3>(1,0,0);
	e2.transform = Eigen::Translation<double, 3>(0,1,0);
	e3.transform = Eigen::Translation<double, 3>(-0.8, -0.7, 0.2);
	e1.covariance = slam::Covariance::Identity();
	e2.covariance = slam::Covariance::Identity();
	e3.covariance = slam::Covariance::Identity();
	solver->addConstraint(e1, v1.id, v2.id);
	solver->addConstraint(e2, v2.id, v3.id);
	solver->addConstraint(e3, v3.id, v1.id);
	BOOST_CHECK_THROW(solver->addConstraint(e3, 3,4), slam::BadEdge);
	
	// Start the optimization
	solver->setFixed(v1.id);
	BOOST_CHECK_THROW(solver->setFixed(99), slam::UnknownVertex);
	
	solver->compute();
}
*/