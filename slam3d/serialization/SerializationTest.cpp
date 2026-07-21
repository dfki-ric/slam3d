#define BOOST_TEST_MODULE "SerializationTest"

#include <boost/test/unit_test.hpp>
#include <slam3d/core/FileLogger.hpp>
#include <slam3d/graph/boost/BoostGraph.hpp>
#include <slam3d/serialization/GraphSerialization.hpp>
#include <slam3d/serialization/MeasurementSerialization.hpp>

#include <filesystem>


//BOOST_CLASS_EXPORT_IMPLEMENT(slam3d::PointCloudMeasurement)

#define ROBOT_NAME "TestRobot"
#define SENSOR_NAME "TestPclSensor"

using namespace slam3d;

class TestMeasurement : public Measurement
{
public:
	typedef boost::shared_ptr<TestMeasurement> Ptr;
	
	TestMeasurement(const std::string& text)
	: mText(text){}
	
	~TestMeasurement() {}
	
	const std::string& getText() const { return mText; }
	
	virtual const char* getTypeName() const override { return "slam3d::TestMeasurement"; }

protected:
	std::string mText;
};

BOOST_AUTO_TEST_CASE(graph_export)
{
	
	Clock clock;
	FileLogger logger(clock, "test_graph_export.log");
	logger.setLogLevel(DEBUG);

	// Fill storage
	MeasurementStorage storage;
	
	TestMeasurement::Ptr m1 = boost::make_shared<TestMeasurement>("Test One");
	TestMeasurement::Ptr m2 = boost::make_shared<TestMeasurement>("Test Two");
	TestMeasurement::Ptr m3 = boost::make_shared<TestMeasurement>("Test Three");
	
	MetaData d1 = initMetaData(clock.now(), m1->getTypeName(), ROBOT_NAME, SENSOR_NAME, Transform::Identity());
	MetaData d2 = initMetaData(clock.now(), m2->getTypeName(), ROBOT_NAME, SENSOR_NAME, Transform::Identity());
	MetaData d3 = initMetaData(clock.now(), m3->getTypeName(), ROBOT_NAME, SENSOR_NAME, Transform::Identity());
	
	storage.add(m1, d1.uniqueId);
	storage.add(m2, d2.uniqueId);
	storage.add(m3, d3.uniqueId);
	
	// Fill graph
	BoostGraph graph(&logger);
//	graph.
	
	// Export
	std::string dir("export_test");
	std::filesystem::create_directory(dir);
	slam3d::GraphSerialization::toFile(&graph, dir+"/graph.yml");
	slam3d::MeasurementSerialization::toDirectory(&storage, dir, true);

	// Import
	slam3d::BoostGraph import_graph(&logger);
	slam3d::MeasurementStorage import_storage;
	slam3d::GraphSerialization::fromFile(&import_graph, dir+"/graph.yml");
	slam3d::MeasurementSerialization::fromDirectory(&import_storage, dir, true);
	
	BOOST_CHECK_EQUAL(graph.getVertices().size(), import_graph.getVertices().size());
	BOOST_CHECK_EQUAL(graph.getEdges().size(), import_graph.getEdges().size());
	BOOST_CHECK_EQUAL(storage.getAllKeys().size(), import_storage.getAllKeys().size());
}
