#include "Types.hpp"

#include <boost/uuid/uuid_generators.hpp>

using namespace slam3d;

Measurement::Measurement(const std::string& r, const std::string& s,
                         const Transform& p, const boost::uuids::uuid id)
{
	mMetaData.robotName = r;
	mMetaData.sensorName = s;
	mMetaData.sensorPose = p;
	mMetaData.inverseSensorPose = p.inverse();
	if(id.is_nil())
		mMetaData.uniqueId = boost::uuids::random_generator()();
	else
		mMetaData.uniqueId = id;
}

// Re-orthogonalize the rotation-matrix
Transform slam3d::orthogonalize(const Transform& t)
{
	Eigen::Quaternion<ScalarType> q(t.linear());
	q.normalize();
	Transform res(t);
	res.linear() = q.toRotationMatrix();
	return res;
}
