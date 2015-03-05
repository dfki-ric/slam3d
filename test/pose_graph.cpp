#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN

#define BOOST_TEST_MODULE slam3d

#include <boost/test/unit_test.hpp>
#include <PoseGraph.hpp>

#include <iostream>

BOOST_AUTO_TEST_CASE(construction)
{
	slam::Measurement m1(1);
	slam::Measurement m2(2);
	slam::Measurement m3(3);
	slam::PoseGraph graph;
	slam::Vertex v1 = graph.addVertex(&m1);
	slam::Vertex v2 = graph.addVertex(&m2);
	slam::Vertex v3 = graph.addVertex(&m3);
	
	slam::Edge e1 = graph.addEdge(v1, v2);
	slam::Edge e2 = graph.addEdge(v2, v3);
	slam::Edge e3 = graph.addEdge(v3, v1);
	
	BOOST_CHECK(graph.getMeasurement(v1) == &m1);
	BOOST_CHECK(graph.getMeasurement(v2) == &m2);
	BOOST_CHECK(graph.getMeasurement(v3) == &m3);
	
	BOOST_CHECK(graph.getMeasurement(v1) != &m2);
	BOOST_CHECK(graph.getMeasurement(v1) != &m3);
	
	slam::AdjacencyRange range = graph.getAdjacentVertices(v2);
	
	std::ofstream file;
	file.open("construction_test.dot");
	graph.dumpGraphViz(file);
	file.close();
}