#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE PclSensorTest

#include "PointCloudSensor.hpp"
#include "FileLogger.hpp"

#include <iostream>
#include <fstream>
#include <boost/test/unit_test.hpp>
#include <boost/format.hpp>

using namespace slam;

PointCloud::Ptr loadFromFile(const std::string& filename)
{
	PointCloud::Ptr cloud(new PointCloud);


	// allocate 4 MB buffer (only ~130*4*4 KB are needed) and read in the file
	int32_t num = 1000000;
	float* data = new float[num];	
	
	// load point cloud
	FILE *stream = fopen(filename.c_str(),"rb");
	assert(stream);
	int32_t points = fread(data,sizeof(float),num,stream)/4;
	fclose(stream);
	
	 // pointers to coordiantes
	float *px = data+0;
	float *py = data+1;
	float *pz = data+2;
	float *pr = data+3;
	
	for(int32_t i = 0; i < points; i++)
	{
		PointType point;
		point.x = *px;
		point.y = *py;
		point.z = *pz;
		point.intensity = *pr;
		cloud->push_back(point);
		px+=4; py+=4; pz+=4; pr+=4;
	}
	
	return cloud;
}

BOOST_AUTO_TEST_CASE(icp)
{
	Clock clock;
	FileLogger logger(clock, "pcl_sensor.log");
	logger.setLogLevel(DEBUG);
	PointCloudSensor pclSensor("TestPclSensor", &logger);
	
	// How to access these files properly?
	PointCloud::Ptr cloud1 = loadFromFile("../test/cloud1.bin");
	PointCloud::Ptr cloud2 = loadFromFile("../test/cloud2.bin");
	
	PointCloudMeasurement m1(cloud1, "cloud1");
	PointCloudMeasurement m2(cloud2, "cloud2");
	
	TransformWithCovariance twc = pclSensor.calculateTransform(&m1, &m2, Transform(Eigen::Translation<double, 3>(0,0,0)));
	std::cout << "Translation:" << std::endl << twc.transform.translation() << std::endl << std::endl;
//	std::cout << "Rotation:" << std::endl << twc.transform.rotation() << std::endl;

//	logger.message(DEBUG, (boost::format("Estimated translation:\n %1%") % twc.transform.translation()).str());
//	logger.message(DEBUG, (boost::format("Estimated rotation:\n %1%") % twc.transform.rotation()).str());

	TransformWithCovariance twc2 = pclSensor.calculateTransform(&m1, &m2, twc.transform);
	std::cout << "Translation:" << std::endl << twc2.transform.translation() << std::endl << std::endl;
//	std::cout << "Rotation:" << std::endl << twc2.transform.rotation() << std::endl;

	TransformWithCovariance twc3 = pclSensor.calculateTransform(&m1, &m2, twc2.transform);
	std::cout << "Translation:" << std::endl << twc3.transform.translation() << std::endl << std::endl;
	
}