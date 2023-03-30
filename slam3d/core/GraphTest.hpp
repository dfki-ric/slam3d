#include <slam3d/core/Graph.hpp>
#include <boost/test/unit_test.hpp>


template <class CONSTRAINT> boost::shared_ptr<CONSTRAINT> addAndGetConstraint(slam3d::Graph* graph, boost::shared_ptr<CONSTRAINT> constraint, slam3d::IdType from, slam3d::IdType to, const std::string& sensor){
	BOOST_CHECK_NO_THROW(graph->addConstraint(from, to, constraint));
	slam3d::EdgeObject query_res;
	BOOST_CHECK_NO_THROW(query_res = graph->getEdge(from, to, sensor));
	return boost::dynamic_pointer_cast<CONSTRAINT>(query_res.constraint);
}


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


	slam3d::GravityConstraint::Ptr c3(new slam3d::GravityConstraint("S3", slam3d::Direction::Identity(), slam3d::Direction::Identity(), slam3d::Covariance<2>::Identity()));
	slam3d::GravityConstraint::Ptr c3_res = addAndGetConstraint(graph, c3, 1, 2, "S3");
	BOOST_CHECK_EQUAL(c3->getCovariance().matrix(), c3_res->getCovariance().matrix());
	BOOST_CHECK_EQUAL(c3->getDirection().matrix(), c3_res->getDirection().matrix());
	BOOST_CHECK_EQUAL(c3->getReference().matrix(), c3_res->getReference().matrix());

	slam3d::PositionConstraint::Ptr c4(new slam3d::PositionConstraint("S4", slam3d::Position::Identity(), slam3d::Covariance<3>::Identity(), slam3d::Transform::Identity()));
	slam3d::PositionConstraint::Ptr c4_res = addAndGetConstraint(graph, c4, 1, 2, "S4");
	BOOST_CHECK_EQUAL(c4->getCovariance().matrix(), c4_res->getCovariance().matrix());
	BOOST_CHECK_EQUAL(c4->getPosition().matrix(), c4_res->getPosition().matrix());
	BOOST_CHECK_EQUAL(c4->getSensorPose().matrix(), c4_res->getSensorPose().matrix());

	slam3d::OrientationConstraint::Ptr c5(new slam3d::OrientationConstraint("S5", slam3d::Quaternion::Identity(), slam3d::Covariance<3>::Identity(), slam3d::Transform::Identity()));
	slam3d::OrientationConstraint::Ptr c5_res = addAndGetConstraint(graph, c5, 1, 2, "S5");
	BOOST_CHECK_EQUAL(c5->getCovariance().matrix(), c5_res->getCovariance().matrix());
	BOOST_CHECK_EQUAL(c5->getOrientation().matrix(), c5_res->getOrientation().matrix());
	BOOST_CHECK_EQUAL(c5->getSensorPose().matrix(), c5_res->getSensorPose().matrix());

}
