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

#include <boost/shared_ptr.hpp>
#include <boost/uuid/nil_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_serialize.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/split_free.hpp>

#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <map>
#include <iostream>


namespace slam3d
{
	typedef unsigned IdType;
	typedef double ScalarType;
	typedef Eigen::Matrix<ScalarType,3,1> Position;
	typedef Eigen::Matrix<ScalarType,3,1> Direction;
	typedef Eigen::Quaternion<ScalarType> Quaternion;
	typedef Eigen::Transform<ScalarType,3,Eigen::Isometry> Transform;
	template <unsigned N> using Covariance = Eigen::Matrix<ScalarType,N,N>;
}

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void save(Archive & ar, const slam3d::Transform &tf, const unsigned int version)
		{
			ar & make_array(tf.matrix().data(), tf.matrix().rows() * tf.matrix().cols());
		}

		template<class Archive>
		void load(Archive & ar, slam3d::Transform &tf, const unsigned int version)
		{
			tf.setIdentity(); // init whole matrix
			ar & make_array(tf.matrix().data(), tf.matrix().rows() * tf.matrix().cols());
		}
	} // namespace serialization
} // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(slam3d::Transform)

namespace slam3d
{
	/**
	 * @brief Re-orthogonalize the rotation-matrix
	 * @param t input tranform
	 * @return the orthogonalized transform
	 */
	Transform orthogonalize(const Transform& t);
	
	/**
	 * @class Indexer
	 * @brief Constructor for continuous identifiers.
	 */
	class Indexer
	{
	public:
		Indexer():mNextID(1) {}
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
		Measurement(const std::string& r, const std::string& s,
		            const Transform& p, const boost::uuids::uuid id = boost::uuids::nil_uuid());
		virtual ~Measurement(){}
		
		timeval getTimestamp() const { return mStamp; }
		std::string getRobotName() const { return mRobotName; }
		std::string getSensorName() const { return mSensorName; }
		boost::uuids::uuid getUniqueId() const { return mUniqueId; }
		Transform getSensorPose() const { return mSensorPose; }
		Transform getInverseSensorPose() const { return mInverseSensorPose; }
		
		virtual const char* getTypeName() const = 0;
		
	protected:
		timeval mStamp;
		std::string mRobotName;
		std::string mSensorName;
		boost::uuids::uuid mUniqueId;
		
		Transform mSensorPose;
		Transform mInverseSensorPose;
	};
	
	enum ConstraintType {TENTATIVE, SE3, GRAVITY, POSITION, ORIENTATION};
	
	/**
	 * @class Constraint
	 * @brief Base class for a constraint in the pose graph.
	 * @details This can be a unary contraint on a pose (e.g. GPS, gravity)
	 * or a binary constraint on two poses (e.g. ICP result)
	 */
	class Constraint
	{
	public:
		typedef boost::shared_ptr<Constraint> Ptr;
		
	public:
		Constraint(const std::string& sensor) : mSensorName(sensor) {}
		virtual ~Constraint(){}
		virtual ConstraintType getType() = 0;
		virtual const char* getTypeName() = 0;

		timeval getTimestamp() const { return mStamp; }
		const std::string& getSensorName() const { return mSensorName; }

	protected:
		timeval mStamp;
		std::string mSensorName;
	};
	
	/**
	 * @class SE3Constraint
	 * @brief 
	 */
	class SE3Constraint : public Constraint
	{
	public:
		typedef boost::shared_ptr<SE3Constraint> Ptr;
		
		SE3Constraint(const std::string& s, const Transform& t, const Covariance<6>& i)
		: Constraint(s), mRelativePose(t), mInformation(i) {}

		ConstraintType getType() { return SE3; }
		const char* getTypeName() { return "SE(3)"; }
		
		const Transform& getRelativePose() const { return mRelativePose; }
		const Covariance<6>& getInformation() const { return mInformation; }
		
	protected:
		Transform mRelativePose;
		Covariance<6> mInformation;
		
	};
	
	/**
	 * @class GravityConstraint
	 * @brief 
	 */
	class GravityConstraint : public Constraint
	{
	public:
		typedef boost::shared_ptr<GravityConstraint> Ptr;
		
		GravityConstraint(const std::string& s, const Direction& d, const Direction& r, const Covariance<2>& c)
		: Constraint(s), mDirection(d), mReference(r), mCovariance(c) {}
		
		ConstraintType getType() { return GRAVITY; }
		const char* getTypeName() { return "Gravity"; }
		
		const Direction& getDirection() const { return mDirection; }
		const Direction& getReference() const { return mReference; }
		const Covariance<2>& getCovariance() const { return mCovariance; }
		
	protected:
		Direction mDirection;
		Direction mReference;
		Covariance<2> mCovariance;
	};
	
	/**
	 * @class PositionConstraint
	 * @brief 
	 */
	class PositionConstraint : public Constraint
	{
	public:
		typedef boost::shared_ptr<PositionConstraint> Ptr;
		
		PositionConstraint(const std::string& s,
		                   const Position& p,
		                   const Covariance<3>& c,
		                   const Transform& t)
		: Constraint(s), mPosition(p), mCovariance(c), mSensorPose(t) {}
		
		ConstraintType getType() { return POSITION; }
		const char* getTypeName() { return "Position"; }
		
		const Position& getPosition() const { return mPosition; }
		const Covariance<3>& getCovariance() const { return mCovariance; }
		const Transform& getSensorPose() const { return mSensorPose; }

	protected:
		Position mPosition;
		Covariance<3> mCovariance;
		Transform mSensorPose;
	};

	/**
	 * @class OrientationConstraint
	 * @brief 
	 */
	class OrientationConstraint : public Constraint
	{
	public:
		typedef boost::shared_ptr<OrientationConstraint> Ptr;
		
		OrientationConstraint(const std::string& s,
		                      const Quaternion& q,
		                      const Covariance<3>& c,
		                      const Transform& t)
		: Constraint(s), mOrientation(q), mCovariance(c), mSensorPose(t) {}
		
		ConstraintType getType() { return ORIENTATION; }
		const char* getTypeName() { return "Orientation"; }
		
		const Quaternion& getOrientation() const { return mOrientation; }
		const Covariance<3>& getCovariance() const { return mCovariance; }
		const Transform& getSensorPose() const { return mSensorPose; }

	protected:
		Quaternion mOrientation;
		Covariance<3> mCovariance;
		Transform mSensorPose;
	};
	
/**
 * @class TentativeConstraint
 * @brief Placeholder for a constraint to be added at a later time
 */
	class TentativeConstraint : public Constraint
	{
	public:
		TentativeConstraint(const std::string& s):  Constraint(s){}
		
		ConstraintType getType() { return TENTATIVE; }
		const char* getTypeName() { return "Tentative"; }
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
		std::string label;
		IdType source;
		IdType target;
		Constraint::Ptr constraint;
	};

	typedef std::vector<VertexObject> VertexObjectList;
	typedef std::vector<EdgeObject> EdgeObjectList;
}

#endif
