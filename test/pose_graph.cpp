#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE slam3d

#include "G2oSolver.hpp"
#include "FileLogger.hpp"
#include "GraphMapper.hpp"

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <graph_analysis/lemon/DirectedGraph.hpp>
#include <graph_analysis/io/GraphvizWriter.hpp>

BOOST_AUTO_TEST_CASE(construction)
{
	
	slam::Clock clock;
	slam::FileLogger logger(clock, "pose_graph_1.log");
	graph_analysis::BaseGraph::Ptr graph(new graph_analysis::lemon::DirectedGraph());
	
	//slam::PoseGraph graph;
	slam::Measurement m1(clock.now(), "Sensor");
	slam::Measurement m2(clock.now(), "Sensor");
	slam::Measurement m3(clock.now(), "Sensor");

	// Create the vertices
	slam::VertexList vertexList;
	slam::VertexObject::Ptr vo1(new slam::VertexObject());
	vo1->measurement = &m1;
	vo1->odometric_pose = Eigen::Translation<double, 3>(0,0,0);
	vo1->corrected_pose = Eigen::Translation<double, 3>(0,0,0);
	graph->addVertex(vo1);

	slam::VertexObject::Ptr vo2(new slam::VertexObject());
	vo2->measurement = &m2;
	vo2->odometric_pose = Eigen::Translation<double, 3>(1,0,0);
	vo2->corrected_pose = Eigen::Translation<double, 3>(1,0,0);
	graph->addVertex(vo2);

	slam::VertexObject::Ptr vo3(new slam::VertexObject());
	vo3->measurement = &m3;
	vo3->odometric_pose = Eigen::Translation<double, 3>(1,1,0);
	vo3->corrected_pose = Eigen::Translation<double, 3>(1,1,0);
	graph->addVertex(vo3);
	
	// Create the edges
	slam::EdgeObject::Ptr e1(new slam::EdgeObject());
	e1->setSourceVertex(vo1);
	e1->setTargetVertex(vo2);
	e1->covariance = slam::Covariance::Identity();
	e1->transform = Eigen::Translation<double, 3>(1,0,0);
	graph->addEdge(e1);
	
	slam::EdgeObject::Ptr e2(new slam::EdgeObject());
	e2->setSourceVertex(vo2);
	e2->setTargetVertex(vo3);
	e2->covariance = slam::Covariance::Identity();
	e2->transform = Eigen::Translation<double, 3>(0,1,0);
	graph->addEdge(e2);
	
	slam::EdgeObject::Ptr e3(new slam::EdgeObject());
	e3->setSourceVertex(vo3);
	e3->setTargetVertex(vo1);
	e3->covariance = slam::Covariance::Identity();
	e3->transform = Eigen::Translation<double, 3>(-0.8, -0.7, 0.2);
	graph->addEdge(e3);

	// Test file output
	graph_analysis::io::GraphIO::write("construction_test.dot", *graph, graph_analysis::representation::GRAPHVIZ);
}
