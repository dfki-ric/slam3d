#include "Types.hpp"

#include <boost/uuid/uuid_generators.hpp>
#include <boost/format.hpp>

using namespace slam3d;

// Re-orthogonalize the rotation-matrix
Transform slam3d::orthogonalize(const Transform& t)
{
	Eigen::Quaternion<ScalarType> q(t.linear());
	q.normalize();
	Transform res(t);
	res.linear() = q.toRotationMatrix();
	return res;
}

MetaData slam3d::initMetaData(timeval time, std::string type, std::string robot, std::string sensor, Transform pose)
{
	MetaData data;
	data.timestamp = time;
	data.typeName = type;
	data.robotName = robot;
	data.sensorName = sensor;
	data.sensorPose = pose;
	data.inverseSensorPose = pose.inverse();
	data.uniqueId = boost::uuids::random_generator()();
	return data;
}

void VertexObject::init(const MetaData& d, IdType i)
{
	measurement = d;
	index = i;
	boost::format frm("%1%:%2%(%3%)");
	frm % measurement.robotName % measurement.sensorName % index;
	label = frm.str();
}
