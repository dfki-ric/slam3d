#include "GpsPoseSensor.hpp"
#include <boost/format.hpp>

using namespace slam3d;

GpsPoseSensor::GpsPoseSensor(const std::string& n, Graph* g, Logger* l)
: PoseSensor(n, g, l)
{
	mHasNewData = false;
}

GpsPoseSensor::~GpsPoseSensor()
{
	
}

void GpsPoseSensor::handleNewVertex(IdType vertex)
{	
	if(!mHasNewData)
	{
		mLogger->message(WARNING, "GPS has no new sample, not adding any edges.");
		return;
	}

//	double diff = mClock.diff(mTimestamp);
	timeval time_meas = mGraph->getMeasurement(vertex)->getTimestamp();
	timeval time_diff;
	timersub(&time_meas, &mTimestamp, &time_diff);
	mLogger->message(INFO, (boost::format("Time-Diff GPS/Scan: %1%.%2%")%time_diff.tv_sec%time_diff.tv_usec).str());
	if(time_diff.tv_sec >= 1 || time_diff.tv_sec <= -1)
	{
		mLogger->message(WARNING, "GPS/Scan is too old, not adding any edges.");
		return;
	}

	slam3d::PositionConstraint::Ptr position(
		new slam3d::PositionConstraint(mName, mPosition, mCovariance * mCovarianceScale, mSensorPose));
	mGraph->addConstraint(vertex, 0, position);
	mHasNewData = false;
}

Transform GpsPoseSensor::getPose(timeval stamp)
{
	Transform pose = Transform::Identity();
	pose.translation() = mPosition;
	return pose;
}

void GpsPoseSensor::update(const timeval& t, const Position& p,
                           const Covariance<3>& c, const Transform& sp)
{
//	if(timercmp(&t, &mTimestamp, <))
//	{
//		mLogger->message(WARNING, "Received old GPS update, which is ignored.");
//		return;
//	}
	mTimestamp = t;
	mPosition = p;
	mCovariance = c;
	mSensorPose = sp;
	mHasNewData = true;
}
