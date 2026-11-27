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
	
	VertexObjectList vertices = graph.getVertices({sensor.getName()});
	BOOST_CHECK_EQUAL(vertices.size(), 1);
	PointCloud::Ptr map;
	BOOST_CHECK_NO_THROW(map = sensor.buildMap(vertices));
}

BOOST_AUTO_TEST_CASE(crop_turned_box)
{
	// a box of 4 x 2 x 1 from its corner, turned 45 degrees about z and moved to (1, 2, 0)
	const Eigen::Vector4f min(0, 0, 0, 1);
	const Eigen::Vector4f max(4, 2, 1, 1);
	const Eigen::Affine3f pose = Eigen::Translation3f(1, 2, 0) * Eigen::AngleAxisf(M_PI / 4, Eigen::Vector3f::UnitZ());

	PointCloud::Ptr cloud(new PointCloud);
	// inside: the middle of the box, near its far corner and near its corner at (0, 2) in box coordinates
	for(const Eigen::Vector3f& local : {Eigen::Vector3f(2, 1, 0.5f), Eigen::Vector3f(3.9f, 1.9f, 0.9f), Eigen::Vector3f(0.1f, 1.9f, 0.5f)})
	{
		const Eigen::Vector3f p = pose * local;
		cloud->push_back(PointType(p.x(), p.y(), p.z()));
	}
	// outside: in the box along the map axes (from the two corners) but not in the turned box,
	// and in the turned box's map-axes hull only
	const Eigen::Vector3f corner = pose * Eigen::Vector3f(4, 2, 1);
	cloud->push_back(PointType(corner.x() - 0.05f, 2.05f, 0.5f));
	cloud->push_back(PointType(1 + 2.0f, 2, 0.5f));

	PointCloud::Ptr turned = PointCloudSensor::crop(cloud, min, max, pose);
	BOOST_CHECK_EQUAL(turned->size(), 3);
	for(size_t i = 0; i < turned->size(); ++i)
		BOOST_CHECK_SMALL((turned->at(i).getVector3fMap() - cloud->at(i).getVector3fMap()).norm(), 1e-6f);

	// without a pose the box is along the axes, (0..4, 0..2, 0..1): only (3, 2, 0.5) is in it
	PointCloud::Ptr along = PointCloudSensor::crop(cloud, min, max);
	BOOST_REQUIRE_EQUAL(along->size(), 1);
	BOOST_CHECK_EQUAL(along->at(0).x, 3.0f);

	// the default map config crops along the map axes, as before the pose existed
	PointCloudSensor::MapConfig config;
	BOOST_CHECK(config.cropBoxPose.matrix().isIdentity());
}
