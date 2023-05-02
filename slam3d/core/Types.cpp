#include "Types.hpp"

#include <boost/uuid/uuid_generators.hpp>

BOOST_CLASS_EXPORT_IMPLEMENT(slam3d::Measurement)

using namespace slam3d;

Measurement::Measurement(const std::string& r, const std::string& s,
                         const Transform& p, const boost::uuids::uuid id)
{
	mRobotName = r;
	mSensorName = s;
	mSensorPose = p;
	mInverseSensorPose = p.inverse();
	if(id.is_nil())
		mUniqueId = boost::uuids::random_generator()();
	else
		mUniqueId = id;
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
