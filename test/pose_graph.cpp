#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE PoseGraphTest

#include <slam3d/FileLogger.hpp>
#include <slam3d/GraphMapper.hpp>

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <graph_analysis/lemon/DirectedGraph.hpp>
#include <graph_analysis/io/GraphvizWriter.hpp>

class TestMeasurement : public slam3d::Measurement
{
public:
	TestMeasurement()
	{
		mRobotName = "Robot";
		mSensorName = "Sensor";
		mSensorPose = slam3d::Transform::Identity();
		mInverseSensorPose = slam3d::Transform::Identity();
	}
};

BOOST_AUTO_TEST_CASE(construction)
{
	slam3d::Clock clock;
	slam3d::FileLogger logger(clock, "pose_graph_1.log");
	graph_analysis::BaseGraph::Ptr graph(new graph_analysis::lemon::DirectedGraph());
	
	// Create the measurements
	TestMeasurement m0, m1, m2, m3, m4, m5;

	// Create the vertices
	slam3d::VertexObject::Ptr vo0(new slam3d::VertexObject("v0"));
	vo0->measurement = &m0;
	graph->addVertex(vo0);
	
	slam3d::VertexList vertexList;
	slam3d::VertexObject::Ptr vo1(new slam3d::VertexObject("v1"));
	vo1->measurement = &m1;
	graph->addVertex(vo1);

	slam3d::VertexObject::Ptr vo2(new slam3d::VertexObject("v2"));
	vo2->measurement = &m2;
	graph->addVertex(vo2);

	slam3d::VertexObject::Ptr vo3(new slam3d::VertexObject("v3"));
	vo3->measurement = &m3;
	graph->addVertex(vo3);
	
	// Create the edges
	slam3d::EdgeObject::Ptr e0(new slam3d::EdgeObject("sensor", "e0"));
	e0->setSourceVertex(vo0);
	e0->setTargetVertex(vo1);
	graph->addEdge(e0);
	
	slam3d::EdgeObject::Ptr e1(new slam3d::EdgeObject("sensor", "e1"));
	e1->setSourceVertex(vo1);
	e1->setTargetVertex(vo2);	
	graph->addEdge(e1);
	
	slam3d::EdgeObject::Ptr e2(new slam3d::EdgeObject("sensor", "e2"));
	e2->setSourceVertex(vo2);
	e2->setTargetVertex(vo3);
	graph->addEdge(e2);
	
	slam3d::EdgeObject::Ptr e3(new slam3d::EdgeObject("sensor", "e3"));
	e3->setSourceVertex(vo3);
	e3->setTargetVertex(vo0);
	graph->addEdge(e3);

	// Test file output
	graph_analysis::io::GraphIO::write("test_01.dot", *graph, graph_analysis::representation::GRAPHVIZ);
	
	// Remove a vertex
	graph_analysis::EdgeIterator::Ptr it = graph->getEdgeIterator(vo2);
	while(it->next())
	{
		graph->removeEdge(it->current());
	}
	graph->removeVertex(vo2);
	
	// Add another one
	slam3d::VertexObject::Ptr vo4(new slam3d::VertexObject());
	vo4->measurement = &m4;
	graph->addVertex(vo4);
	
	slam3d::EdgeObject::Ptr e4(new slam3d::EdgeObject("sensor", "e4"));
	e4->setSourceVertex(vo3);
	e4->setTargetVertex(vo4);
	graph->addEdge(e4);
	
	slam3d::EdgeObject::Ptr e5(new slam3d::EdgeObject("sensor", "e5"));
	e5->setSourceVertex(vo3);
	e5->setTargetVertex(vo4);
	graph->addEdge(e5);
	
	graph_analysis::io::GraphIO::write("test_02.dot", *graph, graph_analysis::representation::GRAPHVIZ);

}
