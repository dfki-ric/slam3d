#define BOOST_TEST_MODULE "PointCloudSensorTest"

#include <boost/test/unit_test.hpp>
#include <pcl/io/ply_io.h>
#include <slam3d/core/Mapper.hpp>
#include <slam3d/core/FileLogger.hpp>
#include <slam3d/core/test_templates/MeasurementTests.hpp>
#include <slam3d/graph/boost/BoostGraph.hpp>

#include <sstream>

#include "PointCloudSensor.hpp"


BOOST_CLASS_EXPORT_IMPLEMENT(slam3d::PointCloudMeasurement)

#define ROBOT_NAME "TestRobot"
#define SENSOR_NAME "TestPclSensor"

using namespace slam3d;

void initEigenTransform(slam3d::Transform* mat)
{
	for (size_t c = 0; c < mat->matrix().cols(); ++c)
		for (size_t r = 0; r < mat->matrix().rows(); ++r)
			(*mat)(r, c) = r+c*mat->matrix().rows();
}

BOOST_AUTO_TEST_CASE(serialization)
{
	Clock clock;
	FileLogger logger(clock, "pcl_sensor_serialization.log");
	logger.setLogLevel(DEBUG);
	PointCloudSensor sensor("pcl-test-sensor", &logger);

	PointCloud::Ptr pcl_cloud(new PointCloud());
	pcl::PLYReader ply_reader;
	std::string test_file(TEST_FILE_PATH);
	test_file += "/test.ply";
	int result = ply_reader.read(test_file, *pcl_cloud);
	pcl_cloud->header.stamp = 1683104082;
	pcl_cloud->header.frame_id = "test_frame";
	pcl_cloud->header.seq = 54321;
	BOOST_CHECK_GE(result, 0);

	if(result >= 0)
	{
		slam3d::Transform tf;
		initEigenTransform(&tf);
		PointCloudMeasurement::Ptr pcl1 = boost::make_shared<PointCloudMeasurement>(pcl_cloud, ROBOT_NAME, SENSOR_NAME, tf);

		Measurement::Ptr pcl2_base = test_serialization(pcl1);

		PointCloudMeasurement::Ptr pcl2 = boost::dynamic_pointer_cast<PointCloudMeasurement>(pcl2_base);
		BOOST_ASSERT(pcl2);
		if(pcl2)
		{
			BOOST_CHECK_EQUAL(pcl1->getPointCloud()->size(), pcl2->getPointCloud()->size());
		}

		// check values of subtype slam3d::PointCloudMeasurement
		BOOST_CHECK_EQUAL(pcl2->getTypeName(), "slam3d::PointCloudMeasurement");
		BOOST_CHECK_EQUAL(pcl2->getPointCloud()->header.seq, 54321);
		BOOST_CHECK_EQUAL(pcl2->getPointCloud()->header.frame_id, "test_frame");

		BOOST_CHECK(pcl1->getPointCloud()->sensor_origin_.isApprox(pcl2->getPointCloud()->sensor_origin_));
		BOOST_CHECK(pcl1->getPointCloud()->sensor_orientation_.isApprox(pcl2->getPointCloud()->sensor_orientation_));
	}
}

BOOST_AUTO_TEST_CASE(map_building)
{
	PointCloud::Ptr pcl_cloud(new PointCloud());
	pcl_cloud->header.stamp = 1683104082;
	pcl_cloud->header.frame_id = "test_frame";
	pcl_cloud->header.seq = 54321;
	
	Clock clock;
	FileLogger logger(clock, "pcl_sensor_map_building.log");
	logger.setLogLevel(DEBUG);

	MeasurementStorage storage;
	BoostGraph graph(&logger, &storage);
	Mapper mapper(&graph, &logger);
	PointCloudSensor sensor("pcl-test-sensor", &logger);

	mapper.registerSensor(&sensor);

	PointCloudMeasurement::Ptr m(new PointCloudMeasurement(pcl_cloud, "robot", sensor.getName(), Transform::Identity()));
	sensor.addMeasurement(m);
	
	VertexObjectList vertices = graph.getVerticesFromSensor(sensor.getName());
	BOOST_CHECK_EQUAL(vertices.size(), 1);
	PointCloud::Ptr map;
	BOOST_CHECK_NO_THROW(map = sensor.buildMap(vertices));
}
