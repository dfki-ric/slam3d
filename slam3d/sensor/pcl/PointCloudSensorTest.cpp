#define BOOST_TEST_MODULE "PointCloudSensorTest"

#include <boost/test/unit_test.hpp>
#include <pcl/io/ply_io.h>
#include <slam3d/core/FileLogger.hpp>

#include "PointCloudSensor.hpp"

#define ROBOT_NAME "TestRobot"
#define SENSOR_NAME "TestPclSensor"

using namespace slam3d;

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
		PointCloudMeasurement m1(pcl_cloud, ROBOT_NAME, SENSOR_NAME, Transform::Identity());
		
		std::stringstream buffer;
		BOOST_CHECK_NO_THROW(m1.toStream(buffer));
		
		Measurement::Ptr m2;
		BOOST_CHECK_NO_THROW(m2 = sensor.createFromStream(ROBOT_NAME, SENSOR_NAME, Transform::Identity(), boost::uuids::nil_uuid(), buffer));
		
		PointCloudMeasurement::Ptr m3 = boost::dynamic_pointer_cast<PointCloudMeasurement>(m2);
		BOOST_ASSERT(m3);

		if(m3)
		{
			BOOST_CHECK_EQUAL(m1.getPointCloud()->size(), m3->getPointCloud()->size());
		}
	}
}
