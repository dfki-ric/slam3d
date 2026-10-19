#define BOOST_TEST_MODULE "PointCloudSensorTest"

#include <boost/test/unit_test.hpp>
#include <pcl/io/ply_io.h>
#include <slam3d/core/Mapper.hpp>
#include <slam3d/core/FileLogger.hpp>
#include <slam3d/core/test_templates/MeasurementTests.hpp>
#include <slam3d/graph/boost/BoostGraph.hpp>

#include <sstream>

#include "PointCloudSensor.hpp"
#include "SplatMeasurement.hpp"


BOOST_CLASS_EXPORT_IMPLEMENT(slam3d::PointCloudMeasurement)
BOOST_CLASS_EXPORT_IMPLEMENT(slam3d::SplatMeasurement)

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

BOOST_AUTO_TEST_CASE(splat_serialization)
{
	// three splats at x = 0, 1, 2
	SplatCloud::Ptr splats(new SplatCloud);
	for(int i = 0; i < 3; ++i)
	{
		SplatPoint splat;
		splat.x = i;
		splat.y = 0;
		splat.z = 0;
		splat.scale_0 = splat.scale_1 = splat.scale_2 = -2.0f;
		splat.rot_0 = 1;
		splat.rot_1 = splat.rot_2 = splat.rot_3 = 0;
		splat.opacity = 0.5f;
		splat.f_dc_0 = i;
		splat.f_dc_1 = 0.1f;
		splat.f_dc_2 = 0.2f;
		splats->push_back(splat);
	}

	PointCloud::Ptr cloud(new PointCloud());
	cloud->push_back(PointType(0, 0, 0));
	cloud->header.stamp = 1683104082;

	slam3d::Transform tf;
	initEigenTransform(&tf);
	SplatMeasurement::Ptr m1 = boost::make_shared<SplatMeasurement>(splats, cloud, ROBOT_NAME, SENSOR_NAME, tf);

	SplatMeasurement::Ptr m2 = boost::dynamic_pointer_cast<SplatMeasurement>(test_serialization(m1));
	BOOST_REQUIRE(m2);
	// used like a point cloud measurement everywhere
	BOOST_CHECK_EQUAL(m2->getTypeName(), "slam3d::PointCloudMeasurement");
	BOOST_CHECK_EQUAL(m2->getPointCloud()->size(), 1);
	BOOST_REQUIRE_EQUAL(m2->getSplats()->size(), 3);
	BOOST_CHECK_EQUAL(m2->getSplats()->width, 3);
	const SplatPoint& splat = m2->getSplats()->at(1);
	BOOST_CHECK_EQUAL(splat.x, 1.0f);
	BOOST_CHECK_EQUAL(splat.scale[2], -2.0f);
	BOOST_CHECK_EQUAL(splat.rot[0], 1.0f);
	BOOST_CHECK_EQUAL(splat.opacity, 0.5f);
	BOOST_CHECK_EQUAL(splat.f_dc[0], 1.0f);
	BOOST_CHECK_EQUAL(splat.f_dc_2, 0.2f);

	// removes the splat at x = 1
	BOOST_CHECK_EQUAL(m2->removeSplatsBySphere(Eigen::Vector3d(1.1, 0, 0), 0.5), 1);
	BOOST_REQUIRE_EQUAL(m2->getSplats()->size(), 2);
	BOOST_CHECK_EQUAL(m2->getSplats()->at(1).x, 2.0f);

	// removes everything but the splat at x = 2
	BOOST_CHECK_EQUAL(m2->removeSplatsBySphere(Eigen::Vector3d(2.1, 0, 0), 0.5, false), 1);
	BOOST_REQUIRE_EQUAL(m2->getSplats()->size(), 1);
	BOOST_CHECK_EQUAL(m2->getSplats()->at(0).x, 2.0f);
	BOOST_CHECK_EQUAL(m2->getSplats()->width, 1);
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
	
	VertexObjectList vertices = graph.getVertices({sensor.getName()});
	BOOST_CHECK_EQUAL(vertices.size(), 1);
	PointCloud::Ptr map;
	BOOST_CHECK_NO_THROW(map = sensor.buildMap(vertices));
}
