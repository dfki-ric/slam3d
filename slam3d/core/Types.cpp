#include "Types.hpp"

#include <boost/uuid/uuid_generators.hpp>

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
