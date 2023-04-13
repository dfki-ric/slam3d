/**
 * @file pcl_serialization.hpp
 * https://github.com/strands-project/v4r/blob/master/modules/common/include/v4r/common/pcl_serialization.h
 
 Copyright (c) 2015, Vision4Robotics group, TU Vienna
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to
 deal in the Software without restriction, including without limitation the
 rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 
 */




#pragma once

#include <Eigen/Core>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLHeader.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{

namespace serialization
{

template<class Archive>
void serialize(Archive & ar, pcl::PCLHeader & g, const unsigned int version)
{
    (void)version;
    ar & g.seq;
    ar & g.stamp;
    ar & g.frame_id;
}

template<class Archive>
void serialize(Archive & ar, pcl::PointXYZ & g, const unsigned int version)
{
    (void)version;
    ar & g.x;
    ar & g.y;
    ar & g.z;
}

template<class Archive>
void serialize(Archive & ar, pcl::PointXYZRGB & g, const unsigned int version)
{
    (void)version;
    ar & g.x;
    ar & g.y;
    ar & g.z;
    ar & g.rgba;
}

template<class Archive>
void serialize(Archive & ar, pcl::PointNormal & g, const unsigned int version)
{
    (void)version;
    ar & g.x;
    ar & g.y;
    ar & g.z;
    ar & g.normal[0];
    ar & g.normal[1];
    ar & g.normal[2];
}

template<class Archive>
void serialize(Archive & ar, pcl::Normal & g, const unsigned int version)
{
    (void)version;
    ar & g.normal[0];
    ar & g.normal[1];
    ar & g.normal[2];
}


template<class Archive>
void serialize(Archive & ar,Eigen::Quaternion<float> & g, const unsigned int version)
{
    (void)version;
    ar & g.w();
    ar & g.x();
    ar & g.y();
    ar & g.z();
}
template<class Archive>
void serialize(Archive & ar, Eigen::Translation<float, 3> & g, const unsigned int version)
{
    (void)version;
    ar & g.x();
    ar & g.y();
    ar & g.z();
}

template<class Archive>
void serialize(Archive & ar, Eigen::UniformScaling<float> & g, const unsigned int version)
{
    (void)version;
    ar & g.factor();
}

template<class Archive>
void serialize(Archive & ar, pcl::PointIndices & g, const unsigned int version)
{
    (void)version;
    ar & g.indices;
    ar & g.header;
}

template<class Archive>
void serialize(Archive & ar, pcl::PointCloud<pcl::PointNormal> & g, const unsigned int version)
{
    (void)version;
    ar & g.header;
    ar & g.points;
    ar & g.height;
    ar & g.width;
    ar & g.is_dense;
}

template<class Archive>
void serialize(Archive & ar, pcl::PointCloud<pcl::PointXYZ> & g, const unsigned int version)
{
    (void)version;
    ar & g.header;
    ar & g.points;
    ar & g.height;
    ar & g.width;
    ar & g.is_dense;
}

template<class Archive>
void serialize(Archive & ar, pcl::PointCloud<pcl::PointXYZRGB> & g, const unsigned int version)
{
    (void)version;
    ar & g.header;
    ar & g.points;
    ar & g.height;
    ar & g.width;
    ar & g.is_dense;
}

template<class Archive>
void serialize(Archive & ar, pcl::PointCloud<pcl::Normal> & g, const unsigned int version)
{
    (void)version;
    ar & g.header;
    ar & g.points;
    ar & g.height;
    ar & g.width;
    ar & g.is_dense;
}

template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize( Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t,const unsigned int file_version)
{
    (void) file_version;
    int rows = t.rows(), cols = t.cols();
    ar & rows;
    ar & cols;
    if( rows * cols != t.size() )
    t.resize( rows, cols );

    for(int i=0; i<t.size(); i++)
    ar & t.data()[i];
}


}
}
