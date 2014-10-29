#ifndef SLAM3D_TYPES_HPP
#define SLAM3D_TYPES_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace slam3d
{
	// Type definitions of various components
	typedef float ScalarType;
	typedef Eigen::Matrix<ScalarType, 4, 4> Pose;
	
	typedef pcl::PointXYZI PointType;
	typedef pcl::PointCloud<PointType> PointCloud;
}

#endif