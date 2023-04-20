#pragma once

#include <Eigen/Core>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLHeader.h>
#include <pcl/conversions.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>


namespace slam3d
{
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> PointCloud;
}

namespace boost {
namespace serialization {

// template<class Archive>
// void serialize(Archive & ar, pcl::PCLPointCloud2 & m, const unsigned int version)
// {
// 	ar & m.header;
// 	ar & m.width;
// 	ar & m.height;
// 	ar & m.is_bigendian;
// 	ar & m.is_dense;
// 	ar & m.point_step;
// 	ar & m.row_step;
// 	ar & m.fields;
// 	ar & m.data;
// }



template<class Archive>
void serialize(Archive & ar, pcl::PCLPointField & f, const unsigned int version)
{
	ar & f.count;
	ar & f.datatype;
	ar & f.name;
	ar & f.offset;
}

template<class Archive>
void serialize(Archive & ar, pcl::PCLHeader & h, const unsigned int version)
{
	ar & h.frame_id;
	ar & h.seq;
	ar & h.stamp;
}

template<class Archive> void serialize(Archive & ar, slam3d::PointType &point, const unsigned int version)
{
    ar & point.x;
    ar & point.y;
    ar & point.z;
}

template<class Archive> void serialize(Archive & ar, Eigen::Vector4f &point, const unsigned int version)
{
    ar & point[0];
    ar & point[1];
    ar & point[2];
    ar & point[3];
}

template<class Archive> void serialize(Archive & ar, Eigen::Quaternionf &point, const unsigned int version)
{
    ar & point.x();
    ar & point.y();
    ar & point.z();
    ar & point.w();
}

// template<class Archive> void save(Archive & ar, slam3d::PointCloud cloud, const unsigned int version)
// {
//     // pcl::PCLPointCloud2 blob;
// 	// pcl::toPCLPointCloud2(cloud, blob);
//     // ar & blob;
// }
template<class Archive> void serialize(Archive & ar, slam3d::PointCloud &cloud, const unsigned int version)
{
    // pcl::PCLPointCloud2 blob;
    // ar & blob;
    // pcl::fromPCLPointCloud2(blob, cloud);

    ar & cloud.header;
    ar & cloud.points;
    ar & cloud.width;
    ar & cloud.height;
    ar & cloud.is_dense;
    ar & cloud.sensor_origin_;
    ar & cloud.sensor_orientation_;

}

}  // namespace serialization
}  // namespace boost

// BOOST_SERIALIZATION_SPLIT_FREE(slam3d::PointCloud)
