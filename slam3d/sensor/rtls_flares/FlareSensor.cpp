#include "FlareSensor.hpp"

using namespace slam3d;

FlareSensor::FlareSensor(Graph* g, Logger* l) : PoseSensor("RTLS-Flares", g, l)
{
	mTimestamp.tv_sec = 0;
	mTimestamp.tv_usec = 0;
	mHasNewData = false;
}

FlareSensor::~FlareSensor()
{
}

void FlareSensor::setStatus(const std::string& json, const Transform& pose)
{
	rtls_flares::Status status(json);
	if(status.mNumberOfUsedAnchors >= 4)
	{
		mStatus = status;
		mTimestamp = mClock.now();
		mSensorPose = pose;
		mHasNewData = true;
	}
}

slam3d::Transform getPoseFromStatus(const rtls_flares::Position& st)
{
	slam3d::Transform tf = slam3d::Transform::Identity();
	tf.translation().x() = st.x;
	tf.translation().y() = st.y;
	tf.translation().z() = st.z;
	return tf;
}

void FlareSensor::handleNewVertex(IdType vertex)
{	
	if(!mHasNewData)
	{
		mLogger->message(WARNING, "RTLS-Flare has no new sample, not adding any edges.");
		return;
	}

	double diff = mClock.diff(mTimestamp);
	if(diff > 1.0)
	{
		mLogger->message(WARNING, "RTLS-Flare status is too old, not adding any edges.");
		return;
	}

	slam3d::Position pos;
	pos(0) = mStatus.mCurrentPosition.x;
	pos(1) = mStatus.mCurrentPosition.y;
	pos(2) = 0;
	slam3d::PositionConstraint::Ptr position(new slam3d::PositionConstraint(
		mName, pos,	slam3d::Covariance<3>::Identity() * mCovarianceScale, mSensorPose));
	mGraph->addConstraint(vertex, 0, position);
	mHasNewData = false;
}

Transform FlareSensor::getPose(timeval stamp)
{
	return getPoseFromStatus(mStatus.mCurrentPosition);
}
