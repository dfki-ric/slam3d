#ifndef SLAM_LINESCANSENSOR_HPP
#define SLAM_LINESCANSENSOR_HPP

#include "Types.hpp"
#include "Sensor.hpp"
#include "GraphMapper.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_2D.h>

namespace slam3d
{
	typedef pcl::PointXYZ PointType;
	typedef pcl::PointCloud<PointType> LineScan;
	typedef pcl::registration::TransformationEstimation2D<PointType, PointType, ScalarType> ScanMatcher;
	
	class LineScanMeasurement : public Measurement
	{
	public:
		LineScanMeasurement(const LineScan::ConstPtr &scan,
		                    const std::string& r, const std::string& s,
		                    const Transform& tr, const boost::uuids::uuid id = boost::uuids::nil_uuid())
		{
			mScan = scan;
			mRobotName = r;
			mSensorName = s;
			mSensorPose = tr;
			mInverseSensorPose = tr.inverse();
			if(id.is_nil())
				mUniqueId = boost::uuids::random_generator()();
			else
				mUniqueId = id;

			// PCL header should contain microseconds
			mStamp.tv_sec  = scan->header.stamp / 1000000;
			mStamp.tv_usec = scan->header.stamp % 1000000;
		}
	
		const LineScan::ConstPtr getScan() const {return mScan;}
	
	protected:
		LineScan::ConstPtr mScan;
	};
	
	class LineScanSensor : public Sensor
	{
	public:
		LineScanSensor(const std::string& n, Logger* l, const Transform& p);
		
		~LineScanSensor();
		
		TransformWithCovariance calculateTransform(Measurement* source, Measurement* target, Transform odometry) const;
		
		LineScan::Ptr getAccumulatedCloud(VertexList vertices);
	protected:
		ScanMatcher mScanMatcher;
	};
}

#endif