#include <slam3d/core/Graph.hpp>
#include <boost/test/unit_test.hpp>

#include <boost/test/unit_test.hpp>

void addVertexToGraph(slam3d::Graph* g, slam3d::IdType exp_id)
{
	slam3d::Measurement::Ptr m(new slam3d::Measurement());
	slam3d::Transform tf = slam3d::Transform::Identity();
	slam3d::IdType id = g->addVertex(m, tf);
	BOOST_CHECK_EQUAL(id, exp_id);

	slam3d::VertexObject query_res;
	BOOST_CHECK_NO_THROW(query_res = g->getVertex(id));
	BOOST_CHECK_EQUAL(query_res.index, exp_id);
}

void test_graph_construction(slam3d::Graph* graph)
{
	addVertexToGraph(graph, 0);
	addVertexToGraph(graph, 1);
	addVertexToGraph(graph, 2);

	slam3d::SE3Constraint::Ptr c(new slam3d::SE3Constraint("A", slam3d::TransformWithCovariance::Identity()));
	BOOST_CHECK_NO_THROW(graph->addConstraint(0, 1, c));
	
	slam3d::EdgeObject query_res;
	BOOST_CHECK_NO_THROW(query_res = graph->getEdge(0,1,"A"));
	BOOST_CHECK_EQUAL(query_res.target, 1);
	
	BOOST_CHECK_NO_THROW(query_res = graph->getEdge(1,0,"A"));
	BOOST_CHECK_EQUAL(query_res.target, 1);
	
	BOOST_CHECK_THROW(graph->getEdge(0,2,"A"), slam3d::InvalidEdge);
}
