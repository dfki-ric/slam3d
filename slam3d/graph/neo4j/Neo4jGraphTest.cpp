#define BOOST_TEST_MODULE "Neo4jGraphTest"

#include "Neo4jGraph.hpp"

#include <slam3d/core/FileLogger.hpp>
#include <slam3d/core/GraphTest.hpp>

using namespace slam3d;

BOOST_AUTO_TEST_CASE(neo4j_graph_construction)
{
	Clock clock;
	FileLogger logger(clock, "neo4j_graph.log");
	logger.setLogLevel(DEBUG);
	Graph* graph = new Neo4jGraph(&logger);
	test_graph_construction(graph);
	delete graph;
}
