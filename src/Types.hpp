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
	typedef Eigen::Matrix<ScalarType,6,6> Covariance;
	
	/**
	 * @class TransformWithCovariance
	 * @brief Transformation with corresponding covariance matrix.
	 */
	struct TransformWithCovariance
	{
		Transform transform;
		Covariance covariance;
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
		Covariance covariance;
		std::string sensor;
		std::string label;
		IdType source;
		IdType target;
	};

	typedef std::vector<VertexObject> VertexObjectList;
	typedef std::vector<EdgeObject> EdgeObjectList;
}

#endif