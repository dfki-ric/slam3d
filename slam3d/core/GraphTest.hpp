#include <slam3d/core/Graph.hpp>
#include <boost/test/unit_test.hpp>

#include <boost/test/unit_test.hpp>

void addVertexToGraph(slam3d::Graph* g, slam3d::IdType exp_id, const std::string& robot, const std::string& sensor)
{
	slam3d::Measurement::Ptr m(new slam3d::Measurement(robot, sensor, slam3d::Transform::Identity()));
	slam3d::Transform tf = slam3d::Transform::Identity();
	slam3d::IdType id = g->addVertex(m, tf);
	BOOST_CHECK_EQUAL(id, exp_id);

	slam3d::VertexObject query_res;
	BOOST_CHECK_NO_THROW(query_res = g->getVertex(id));
	BOOST_CHECK_EQUAL(query_res.index, exp_id);
}

void test_graph_construction(slam3d::Graph* graph)
{
	addVertexToGraph(graph, 1, "R1", "S1");
	addVertexToGraph(graph, 2, "R1", "S1");
	addVertexToGraph(graph, 3, "R1", "S2");

	slam3d::SE3Constraint::Ptr c1(new slam3d::SE3Constraint("S1", slam3d::Transform::Identity(), slam3d::Covariance<6>::Identity()));
	BOOST_CHECK_NO_THROW(graph->addConstraint(1, 2, c1));
	
	slam3d::SE3Constraint::Ptr c2(new slam3d::SE3Constraint("S2", slam3d::Transform::Identity(), slam3d::Covariance<6>::Identity()));
	BOOST_CHECK_NO_THROW(graph->addConstraint(2, 3, c2));
	
	slam3d::EdgeObject query_res;
	BOOST_CHECK_NO_THROW(query_res = graph->getEdge(1,2,"S1"));
	BOOST_CHECK_EQUAL(query_res.target, 2);
	
	BOOST_CHECK_NO_THROW(query_res = graph->getEdge(2,1,"S1"));
	BOOST_CHECK_EQUAL(query_res.target, 2);
	
	BOOST_CHECK_THROW(graph->getEdge(1,3,"A"), slam3d::InvalidEdge);
	
	slam3d::EdgeObjectList s1_edges;
	BOOST_CHECK_NO_THROW(s1_edges = graph->getEdgesFromSensor("S1"));
	BOOST_CHECK_EQUAL(s1_edges.size(), 1);
	BOOST_CHECK_EQUAL(s1_edges.at(0).source, 1);
	BOOST_CHECK_EQUAL(s1_edges.at(0).target, 2);
}
