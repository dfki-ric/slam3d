#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE slam3d

#include <boost/test/unit_test.hpp>
#include <PoseGraph.hpp>
#include <G2oSolver.hpp>

#include <iostream>

BOOST_AUTO_TEST_CASE(pose_graph_1)
{
	slam::PoseGraph graph;
	slam::Measurement m1(1);
	slam::Measurement m2(2);
	slam::Measurement m3(3);

	slam::VertexObject v_obj;
	v_obj.measurement = &m1;
	slam::Vertex v1 = graph.addVertex(v_obj);
	v_obj.measurement = &m2;
	slam::Vertex v2 = graph.addVertex(v_obj);
	v_obj.measurement = &m3;
	slam::Vertex v3 = graph.addVertex(v_obj);
	
	slam::EdgeObject e_obj;
	graph.addEdge(v1, v2, e_obj);
	graph.addEdge(v2, v3, e_obj);
	graph.addEdge(v3, v1, e_obj);
	
	BOOST_CHECK(graph.getMeasurement(v1) == &m1);
	BOOST_CHECK(graph.getMeasurement(v2) == &m2);
	BOOST_CHECK(graph.getMeasurement(v3) == &m3);
	
	BOOST_CHECK(graph.getMeasurement(v1) != &m2);
	BOOST_CHECK(graph.getMeasurement(v1) != &m3);
	
//	slam::AdjacencyRange range = graph.getAdjacentVertices(v2);
	
	std::ofstream file;
	file.open("construction_test.dot");
	graph.dumpGraphViz(file);
	file.close();
}

BOOST_AUTO_TEST_CASE(g2o_solver_1)
{
	slam::Logger logger;
	slam::G2oSolver* solver = new slam::G2oSolver(&logger);
	
	// Create the nodes
	slam::VertexObject v1, v2, v3;
	v1.corrected_pose = Eigen::Translation<double, 3>(0,0,0);
	v2.corrected_pose = Eigen::Translation<double, 3>(0,0,0);
	v3.corrected_pose = Eigen::Translation<double, 3>(0,0,0);
	solver->addNode(v1, 0);
	solver->addNode(v2, 1);
	solver->addNode(v3, 2);
	BOOST_CHECK_THROW(solver->addNode(v3, 2), slam::DuplicateVertex);

	// Create the edges
	slam::EdgeObject e1,e2, e3;
	e1.transform = Eigen::Translation<double, 3>(1,0,0);
	e2.transform = Eigen::Translation<double, 3>(0,1,0);
	e3.transform = Eigen::Translation<double, 3>(-0.8, -0.7, 0.2);
	e1.covariance = slam::Covariance::Identity();
	e2.covariance = slam::Covariance::Identity();
	e3.covariance = slam::Covariance::Identity();
	solver->addConstraint(e1, 0,1);
	solver->addConstraint(e2, 1,2);
	solver->addConstraint(e3, 2,0);
	BOOST_CHECK_THROW(solver->addConstraint(e3, 3,4), slam::BadEdge);
	
	solver->compute();
}