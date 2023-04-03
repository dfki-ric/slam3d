#define BOOST_TEST_MODULE "Neo4jGraphTest"

#include <slam3d/core/FileLogger.hpp>
#include <slam3d/core/GraphTest.hpp>

#define private public
#define protected public

#include "Neo4jGraph.hpp"
#include "Neo4jConversion.hpp"


using namespace slam3d;

std::unique_ptr<Neo4jGraph> neo4jgraph;
Clock neo4jclock;
FileLogger neo4jlogger(neo4jclock, "neo4j_graph.log");

void initTransform(slam3d::Transform* mat) {
    for (size_t c = 0; c < mat->matrix().cols(); ++c) {
        for (size_t r = 0; r < mat->matrix().rows(); ++r) {
            (*mat)(r, c) = r+c*mat->matrix().rows();
        }
    }
}

void initEigenMatrix(Eigen::MatrixXd* mat) {
    for (size_t c = 0; c < mat->matrix().cols(); ++c) {
        for (size_t r = 0; r < mat->matrix().rows(); ++r) {
            (*mat)(r, c) = r+c*mat->matrix().rows();
        }
    }
}

void initDB() {
    static bool initialized = false;
    if (!initialized) {
        neo4jlogger.setLogLevel(DEBUG);
        neo4jgraph = std::make_unique<Neo4jGraph>(&neo4jlogger);
        neo4jgraph->deleteDatabase();
        initialized = true;
    }
}

BOOST_AUTO_TEST_CASE(neo4j_graph_construction) {
    initDB();
    test_graph_construction(neo4jgraph.get());
}

BOOST_AUTO_TEST_CASE(matrix_conversion) {
    initDB();
    slam3d::Transform t;
    initTransform(&t);
    std::string ts = Neo4jConversion::eigenMatrixToString(t.matrix());
    slam3d::Transform tr(Eigen::Matrix4d(Neo4jConversion::eigenMatrixFromString(ts)));
    BOOST_CHECK_EQUAL(t.matrix(), tr.matrix());
}

BOOST_AUTO_TEST_CASE(contraint_conversion) {
    
    // neo4jgraph->co
}