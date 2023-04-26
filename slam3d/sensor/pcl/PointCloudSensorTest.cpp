#define BOOST_TEST_MODULE "PointCloudSensorTest"

#include <boost/test/unit_test.hpp>
#include <pcl/io/ply_io.h>
#include <slam3d/core/FileLogger.hpp>

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

	if(result >= 0)
	{
		slam3d::Transform tf;
		initEigenTransform(&tf);
		PointCloudMeasurement m(pcl_cloud, ROBOT_NAME, SENSOR_NAME, tf);
		
		std::stringstream buffer;
		boost::archive::text_oarchive oa(buffer);
		oa << m;
		//create data string with the binary data
		std::string data = buffer.str();


		PointCloudMeasurement::Ptr m2 (new PointCloudMeasurement());
		std::stringstream buffer2(data);
		boost::archive::text_iarchive ia(buffer2);
		ia >> *(m2.get());

		BOOST_ASSERT(m2);

		if(m2)
		{
			BOOST_CHECK_EQUAL(m.getPointCloud()->size(), m2->getPointCloud()->size());
		}

		BOOST_CHECK_EQUAL(m.getRobotName(), m2->getRobotName());
		BOOST_CHECK_EQUAL(m.getTimestamp().tv_sec, m2->getTimestamp().tv_sec);
		BOOST_CHECK_EQUAL(m.getTimestamp().tv_usec, m2->getTimestamp().tv_usec);
		BOOST_CHECK_EQUAL(m.getSensorName(), m2->getSensorName());
		BOOST_CHECK(m.getUniqueId() == m2->getUniqueId());
		BOOST_CHECK(m.getSensorPose().isApprox(m2->getSensorPose()));
		BOOST_CHECK(m.getInverseSensorPose().isApprox(m2->getInverseSensorPose()));
		
		// check values of subtype slam3d::PointCloudMeasurement
		BOOST_CHECK_EQUAL(m2->getTypeName(), "slam3d::PointCloudMeasurement");
		BOOST_CHECK_NE(m2->getPointCloud(), nullptr);
		BOOST_CHECK_EQUAL(m2->getPointCloud()->size(), pcl_cloud->size());

		BOOST_CHECK(m.getPointCloud()->sensor_origin_.isApprox(m2->getPointCloud()->sensor_origin_));
		BOOST_CHECK(m.getPointCloud()->sensor_orientation_.isApprox(m2->getPointCloud()->sensor_orientation_));

		// debug out (run with --log_level=message)
		BOOST_TEST_MESSAGE( m.getSensorPose().matrix() );
		BOOST_TEST_MESSAGE( m2->getSensorPose().matrix() );
		BOOST_TEST_MESSAGE( m.getPointCloud()->sensor_origin_.matrix() );
		BOOST_TEST_MESSAGE( m2->getPointCloud()->sensor_origin_.matrix() );


	}
}
