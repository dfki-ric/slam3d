#define BOOST_TEST_MODULE "Neo4jGraphTest"

#include "Neo4jGraph.hpp"

#include <slam3d/core/FileLogger.hpp>
#include <slam3d/core/GraphTest.hpp>

using namespace slam3d;

BOOST_AUTO_TEST_CASE(neo4j_graph_construction) {
    Clock clock;
    FileLogger logger(clock, "neo4j_graph.log");
    logger.setLogLevel(DEBUG);
    Neo4jGraph* neograph = new Neo4jGraph(&logger);
    neograph->deleteDatabase();
    test_graph_construction(neograph);
    delete neograph;
}
