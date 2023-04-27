#define BOOST_TEST_MODULE "Neo4jGraphTest"

#include <slam3d/core/FileLogger.hpp>
#include <slam3d/core/test_templates/GraphTest.hpp>

#define private public
#define protected public

#include <slam3d/measurements/redis/RedisMeasurementStorage.hpp>
#include "Neo4jGraph.hpp"
#include "Neo4jConversion.hpp"

#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <sstream>

using namespace slam3d;

std::unique_ptr<Neo4jGraph> neo4jgraph;
Clock neo4jclock;
FileLogger neo4jlogger(neo4jclock, "neo4j_graph.log");



void initEigenTransform(slam3d::Transform* mat) {
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

void initEigenVector(Eigen::Vector4f* mat) {
    // for (size_t c = 0; c < mat->matrix().cols(); ++c) {
        for (size_t r = 0; r < mat->matrix().rows(); ++r) {
            (*mat)(r) = r;
        }
    // }
}

void initEigenQuaternion(Eigen::Quaternionf* q) {
    q->x() = 1;
    q->y() = 2;
    q->z() = 3;
    q->w() = 4;
}

void initDB() {
    static bool initialized = false;
    if (!initialized) {
        std::shared_ptr<slam3d::MeasurementStorage> measurements = std::make_shared<slam3d::RedisMeasurementStorage>("localhost", 6379);
        neo4jlogger.setLogLevel(DEBUG);
        neo4jgraph = std::make_unique<Neo4jGraph>(&neo4jlogger, measurements);
        neo4jgraph->deleteDatabase();
        initialized = true;
    }
}

void initRegistry() {
    MeasurementSerialization::registerMeasurementType<slam3d::PointCloudMeasurement>("slam3d::PointCloudMeasurement");
}

BOOST_AUTO_TEST_CASE(neo4j_graph_construction) {
    initDB();
    initRegistry();
    test_graph_construction(neo4jgraph.get());
}

BOOST_AUTO_TEST_CASE(matrix_conversion) {
    initDB();
    slam3d::Transform t;
    initEigenTransform(&t);
    std::string ts = Neo4jConversion::eigenMatrixToString(t.matrix());
    slam3d::Transform tr(Eigen::Matrix4d(Neo4jConversion::eigenMatrixFromString(ts)));
    BOOST_CHECK_EQUAL(t.matrix(), tr.matrix());
}

BOOST_AUTO_TEST_CASE(contraint_conversion) {
    // neo4jgraph->co
}


BOOST_AUTO_TEST_CASE(measurement_serialization) {

    PointCloud::Ptr cloud = PointCloud::Ptr(new PointCloud());

    cloud->push_back(slam3d::PointType(1, 2, 3));

    slam3d::Transform tf;
    // fill some comparable vlaues (identity will be same in initialized but not serialized)
    initEigenVector(&(cloud->sensor_origin_));
    initEigenQuaternion(&cloud->sensor_orientation_);
    initEigenTransform(&tf);

    slam3d::PointCloudMeasurement::Ptr m(new slam3d::PointCloudMeasurement(cloud, "robot", "sensor", tf));

    std::stringstream sin;
    boost::archive::text_oarchive oa(sin);
    // TODO, need to check for sub-type? https://theboostcpplibraries.com/boost.serialization-class-hierarchies
    oa << *(m.get());
    std::string data = sin.str();

    std::stringstream sout(data);
    boost::archive::text_iarchive ia(sout);
    PointCloudMeasurement::Ptr m_res = PointCloudMeasurement::Ptr(new PointCloudMeasurement());
    ia >> *(m_res.get());

    // ceck values of base slam3d::Measurement 
    BOOST_CHECK_NE(m_res.get(), nullptr);
    BOOST_CHECK_EQUAL(m->getRobotName(), m_res->getRobotName());
    BOOST_CHECK_EQUAL(m->getTimestamp().tv_sec, m_res->getTimestamp().tv_sec);
    BOOST_CHECK_EQUAL(m->getTimestamp().tv_usec, m_res->getTimestamp().tv_usec);
    BOOST_CHECK_EQUAL(m->getSensorName(), m_res->getSensorName());
    BOOST_CHECK(m->getUniqueId() == m_res->getUniqueId());
    BOOST_CHECK(m->getSensorPose().isApprox(m_res->getSensorPose()));
    BOOST_CHECK(m->getInverseSensorPose().isApprox(m_res->getInverseSensorPose()));
    
    // check values of subtype slam3d::PointCloudMeasurement
    BOOST_CHECK_EQUAL(m_res->getTypeName(), "slam3d::PointCloudMeasurement");
    BOOST_CHECK_NE(m_res->getPointCloud(), nullptr);
    BOOST_CHECK_EQUAL(m_res->getPointCloud()->size(), cloud->size());

    BOOST_CHECK(m->getPointCloud()->sensor_origin_.isApprox(m_res->getPointCloud()->sensor_origin_));
    BOOST_CHECK(m->getPointCloud()->sensor_orientation_.isApprox(m_res->getPointCloud()->sensor_orientation_));

    // debug out (run with --log_level=message)
    BOOST_TEST_MESSAGE( m->getSensorPose().matrix() );
    BOOST_TEST_MESSAGE( m_res->getSensorPose().matrix() );
    BOOST_TEST_MESSAGE( m->getPointCloud()->sensor_origin_.matrix() );
    BOOST_TEST_MESSAGE( m_res->getPointCloud()->sensor_origin_.matrix() );
    


}


BOOST_AUTO_TEST_CASE(measurement_storage) {
    initDB();
    initRegistry();

    PointCloud::Ptr cloud = PointCloud::Ptr(new PointCloud());

    cloud->push_back(slam3d::PointType(1, 2, 3));

    slam3d::Graph* g = dynamic_cast<slam3d::Graph*>(neo4jgraph.get());

    //todo add data to cloud

	slam3d::Measurement::Ptr m(new slam3d::PointCloudMeasurement(cloud, "robot", "sensor", slam3d::Transform::Identity()));

	slam3d::Transform tf = slam3d::Transform::Identity();
	slam3d::IdType id = g->addVertex(m, tf);
	//BOOST_CHECK_EQUAL(id, exp_id);

	slam3d::VertexObject query_res;
	BOOST_CHECK_NO_THROW(query_res = g->getVertex(id));
	// BOOST_CHECK_EQUAL(query_res.index, exp_id);

    BOOST_CHECK_EQUAL(m->getRobotName(), query_res.measurement->getRobotName()); 

    slam3d::PointCloudMeasurement::Ptr m_res = boost::dynamic_pointer_cast<slam3d::PointCloudMeasurement>(query_res.measurement);

    BOOST_CHECK_NE(m_res.get(), nullptr);

    BOOST_CHECK_EQUAL(m_res->getPointCloud()->size(), cloud->size());

}