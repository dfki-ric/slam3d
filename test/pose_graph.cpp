#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN

#define BOOST_TEST_MODULE slam3d

#include <boost/test/unit_test.hpp>
#include <PoseGraph.hpp>

BOOST_AUTO_TEST_CASE(add_vertex)
{
	slam::Measurement m(0);
	slam::PoseGraph graph;
	slam::Vertex va = graph.addVertex(&m);
}