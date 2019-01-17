#define BOOST_TEST_MODULE "BoostGraphTest"

#include "BoostGraph.hpp"

#include <slam3d/core/FileLogger.hpp>

#include <iostream>
#include <boost/test/unit_test.hpp>

using namespace slam3d;

void addVertexToGraph(Graph* g, IdType exp_id)
{
	Measurement::Ptr m(new Measurement());
	Transform tf = Transform::Identity();
	IdType id = g->addVertex(m, tf);
	BOOST_CHECK_EQUAL(id, exp_id);

	VertexObject query_res;
	BOOST_CHECK_NO_THROW(query_res = g->getVertex(id));
	BOOST_CHECK_EQUAL(query_res.index, exp_id);
}

BOOST_AUTO_TEST_CASE(graph_editing)
{
	Clock clock;
	FileLogger logger(clock, "boost_graph.log");
	logger.setLogLevel(DEBUG);
	Graph* graph = new BoostGraph(&logger);

	addVertexToGraph(graph, 0);
	addVertexToGraph(graph, 1);
	addVertexToGraph(graph, 2);

	SE3Constraint::Ptr c(new SE3Constraint("A", TransformWithCovariance::Identity()));
	BOOST_CHECK_NO_THROW(graph->addConstraint(0, 1, c));
	
	EdgeObject query_res;
	BOOST_CHECK_NO_THROW(query_res = graph->getEdge(0,1,"A"));
	BOOST_CHECK_EQUAL(query_res.target, 1);
	
	BOOST_CHECK_NO_THROW(query_res = graph->getEdge(1,0,"A"));
	BOOST_CHECK_EQUAL(query_res.target, 1);
	
	BOOST_CHECK_THROW(graph->getEdge(0,2,"A"), slam3d::InvalidEdge);
	
	delete graph;
}
