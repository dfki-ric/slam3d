#ifndef SLAM3D_TYPES_HPP
#define SLAM3D_TYPES_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace slam3d
{	
	// Type definitions of various components
	typedef float ScalarType;
	
	typedef Eigen::Matrix<ScalarType, 4, 4> Pose;
	typedef Eigen::Matrix<ScalarType, 3, 1> Translation;
	typedef Eigen::Matrix<ScalarType, 3, 3> Rotation;
	typedef Eigen::Transform<ScalarType, 3, Eigen::Affine> Affine;
	
	typedef pcl::PointXYZI PointType;
	typedef pcl::PointCloud<PointType> PointCloud;
	typedef pcl::KdTreeFLANN<PointType> SearchTree;
}

#endif