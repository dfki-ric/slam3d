#define BOOST_TEST_MODULE "PointCloudSensorTest"

#include <boost/test/unit_test.hpp>
#include <pcl/io/ply_io.h>
#include <slam3d/core/FileLogger.hpp>
#include <slam3d/core/MeasurementSerialization.hpp>

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
	Clock clock;
	FileLogger logger(clock, "pcl_sensor.log");
	logger.setLogLevel(DEBUG);
	PointCloudSensor sensor("pcl-test-sensor", &logger);

	PointCloud::Ptr pcl_cloud(new PointCloud());
	pcl::PLYReader ply_reader;
	int result = ply_reader.read("../../../../test/test.ply", *pcl_cloud);
	BOOST_CHECK_GE(result, 0);

	MeasurementSerialization registry;
	registry.registerMeasurementType<slam3d::PointCloudMeasurement>("slam3d::PointCloudMeasurement");

	if(result >= 0)
	{
		slam3d::Transform tf;
		initEigenTransform(&tf);
		Measurement::Ptr m1(new PointCloudMeasurement(pcl_cloud, ROBOT_NAME, SENSOR_NAME, tf));
		
		std::string buffer = registry.serialize(m1);
		Measurement::Ptr m2 = registry.deserialize(buffer, "slam3d::PointCloudMeasurement");

		PointCloudMeasurement::Ptr pcl1 = boost::dynamic_pointer_cast<slam3d::PointCloudMeasurement>(m1);
		PointCloudMeasurement::Ptr pcl2 = boost::dynamic_pointer_cast<slam3d::PointCloudMeasurement>(m2);

		BOOST_ASSERT(pcl1);
		BOOST_ASSERT(pcl2);

		if(!pcl1 || !pcl2)
		{
			return;
		}

		BOOST_CHECK_EQUAL(pcl1->getPointCloud()->size(), pcl2->getPointCloud()->size());
		BOOST_CHECK_EQUAL(pcl1->getRobotName(), pcl2->getRobotName());
		BOOST_CHECK_EQUAL(pcl1->getTimestamp().tv_sec, pcl2->getTimestamp().tv_sec);
		BOOST_CHECK_EQUAL(pcl1->getTimestamp().tv_usec, pcl2->getTimestamp().tv_usec);
		BOOST_CHECK_EQUAL(pcl1->getSensorName(), pcl2->getSensorName());
		BOOST_CHECK(pcl1->getUniqueId() == pcl2->getUniqueId());
		BOOST_CHECK(pcl1->getSensorPose().isApprox(pcl2->getSensorPose()));
		BOOST_CHECK(pcl1->getInverseSensorPose().isApprox(pcl2->getInverseSensorPose()));
		
		// check values of subtype slam3d::PointCloudMeasurement
		BOOST_CHECK_EQUAL(m2->getTypeName(), "slam3d::PointCloudMeasurement");

		BOOST_CHECK(pcl1->getPointCloud()->sensor_origin_.isApprox(pcl2->getPointCloud()->sensor_origin_));
		BOOST_CHECK(pcl1->getPointCloud()->sensor_orientation_.isApprox(pcl2->getPointCloud()->sensor_orientation_));

		// debug out (run with --log_level=message)
		BOOST_TEST_MESSAGE( buffer );
	}
}
