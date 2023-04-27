#define BOOST_TEST_MODULE "PointCloudSensorTest"

#include <boost/test/unit_test.hpp>
#include <pcl/io/ply_io.h>
#include <slam3d/core/FileLogger.hpp>
#include <slam3d/core/MeasurementSerialization.hpp>
#include <slam3d/core/test_templates/MeasurementTests.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <sstream>

#include "PointCloudSensor.hpp"

#define ROBOT_NAME "TestRobot"
#define SENSOR_NAME "TestPclSensor"

using namespace slam3d;

void initEigenTransform(slam3d::Transform* mat)
{
	for (size_t c = 0; c < mat->matrix().cols(); ++c)
		for (size_t r = 0; r < mat->matrix().rows(); ++r)
			(*mat)(r, c) = r+c*mat->matrix().rows();
}

BOOST_AUTO_TEST_CASE(pcl_serialization)
{
	// register serialization handler for slam3d::PointCloudMeasurement
	slam3d::MeasurementSerialization::registerMeasurementType<slam3d::PointCloudMeasurement>("slam3d::PointCloudMeasurement");

	Clock clock;
	FileLogger logger(clock, "pcl_sensor.log");
	logger.setLogLevel(DEBUG);
	PointCloudSensor sensor("pcl-test-sensor", &logger);

	PointCloud::Ptr pcl_cloud(new PointCloud());
	pcl::PLYReader ply_reader;
	int result = ply_reader.read("../../../../test/test.ply", *pcl_cloud);
	BOOST_CHECK_GE(result, 0);

	if(result >= 0)
	{
		slam3d::Transform tf;
		initEigenTransform(&tf);
		PointCloudMeasurement::Ptr m = boost::make_shared<PointCloudMeasurement>(pcl_cloud, ROBOT_NAME, SENSOR_NAME, tf);

		Measurement::Ptr m2_base = test_serialization(m);

		PointCloudMeasurement::Ptr m2 = boost::dynamic_pointer_cast<PointCloudMeasurement>(m2_base);
		BOOST_ASSERT(m2);
		if(m2)
		{
			BOOST_CHECK_EQUAL(m->getPointCloud()->size(), m2->getPointCloud()->size());
		}

		// check values of subtype slam3d::PointCloudMeasurement
		BOOST_CHECK_EQUAL(m2->getTypeName(), "slam3d::PointCloudMeasurement");
		BOOST_CHECK_NE(m2->getPointCloud(), nullptr);
		BOOST_CHECK_EQUAL(m2->getPointCloud()->size(), pcl_cloud->size());

		BOOST_CHECK(m->getPointCloud()->sensor_origin_.isApprox(m2->getPointCloud()->sensor_origin_));
		BOOST_CHECK(m->getPointCloud()->sensor_orientation_.isApprox(m2->getPointCloud()->sensor_orientation_));

		// debug out (run with --log_level=message)
		BOOST_TEST_MESSAGE( m->getSensorPose().matrix() );
		BOOST_TEST_MESSAGE( m2->getSensorPose().matrix() );
		BOOST_TEST_MESSAGE( m->getPointCloud()->sensor_origin_.matrix() );
		BOOST_TEST_MESSAGE( m2->getPointCloud()->sensor_origin_.matrix() );


	}
}
