// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2017 S. Kasperski
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SLAM_TYPES_HPP
#define SLAM_TYPES_HPP

#include <sys/time.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <map>

namespace slam3d
{
	typedef unsigned IdType;
	typedef double ScalarType;
	typedef Eigen::Matrix<ScalarType,3,1> Vector3;
	typedef Eigen::Transform<ScalarType,3,Eigen::Isometry> Transform;
	template <unsigned N> using Covariance = Eigen::Matrix<ScalarType,N,N>;
	
	/**
	 * @class TransformWithCovariance
	 * @brief Transformation with corresponding covariance matrix.
	 */
	struct TransformWithCovariance
	{
		Transform transform;
		Covariance<6> covariance;

		TransformWithCovariance() : transform(Transform::Identity()), covariance(Covariance<6>::Zero()) {}
		TransformWithCovariance(const Transform& t, const Covariance<6>& cov) : transform(t), covariance(cov) {}
		static TransformWithCovariance Identity() {return TransformWithCovariance();}
	};
	
	/**
	 * @class Indexer
	 * @brief Constructor for continuous identifiers.
	 */
	class Indexer
	{
	public:
		Indexer():mNextID(0) {}
		IdType getNext() { return mNextID++; }
	private:
		IdType mNextID;
	};
	
	/**
	 * @class Measurement
	 * @brief Base class for a single reading from a sensor.
	 * @details This can be a single laser scan, a point cloud, an image etc.
	 * It can either hold the actual data or contain a pointer to the data,
	 * in case that data management is handled separately from the mapping.
	 */
	class Measurement
	{
	public:
		typedef boost::shared_ptr<Measurement> Ptr;
		
	public:
		Measurement(){}
		virtual ~Measurement(){}
		
		timeval getTimestamp() const { return mStamp; }
		std::string getRobotName() const { return mRobotName; }
		std::string getSensorName() const { return mSensorName; }
		boost::uuids::uuid getUniqueId() const { return mUniqueId; }
		Transform getSensorPose() const { return mSensorPose; }
		Transform getInverseSensorPose() const { return mInverseSensorPose; }
		
	protected:
		timeval mStamp;
		std::string mRobotName;
		std::string mSensorName;
		boost::uuids::uuid mUniqueId;
		
		Transform mSensorPose;
		Transform mInverseSensorPose;
	};
	
	/**
	 * @class MapOrigin
	 * @brief Placeholder for root node in the graph.
	 */
	class MapOrigin : public Measurement
	{
	public:
		MapOrigin()
		{
			mRobotName = "none";
			mSensorName = "none";
			mSensorPose = Transform::Identity();
			mInverseSensorPose = Transform::Identity();
			mUniqueId = boost::uuids::nil_uuid();
		}
	};
	
	/**
	 * @struct VertexObject
	 * @brief Object attached to a vertex in the pose graph.
	 * @details It contains a pointer to an abstract measurement, which could
	 * be anything, e.g. a range scan, point cloud or image.
	 */
	struct VertexObject
	{
		IdType index;
		std::string label;
		Transform corrected_pose;
		Measurement::Ptr measurement;
	};

	/**
	 * @struct EdgeObject
	 * @brief Object attached to an edge in the pose graph.
	 * @details It contains the relative transform from source to target,
	 * the associated covariance matrix and the name of the sensor that
	 * created this spatial relationship.
	 */
	struct EdgeObject
	{
		Transform transform;
		Covariance<6> covariance;
		std::string sensor;
		std::string label;
		IdType source;
		IdType target;
	};

	typedef std::vector<VertexObject> VertexObjectList;
	typedef std::vector<EdgeObject> EdgeObjectList;
}

#endif