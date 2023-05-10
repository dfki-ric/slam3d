#pragma once

#include <slam3d/core/PoseSensor.hpp>
#include <slam3d/sensor/rtls_flares/Status.hpp>

namespace slam3d
{	
	class FlareSensor : public PoseSensor
	{
	public:
		FlareSensor(Graph* g, Logger* l);
		~FlareSensor();

		void handleNewVertex(IdType vertex);
		
		Transform getPose(timeval stamp);
		Transform getSensorPose() { return mSensorPose; }
		
		void setStatus(const std::string& json, const Transform& pose = Transform::Identity());

	protected:
		rtls_flares::Status mStatus;
		Clock mClock;
		timeval mTimestamp;
		Transform mSensorPose;
		bool mHasNewData;
	};
}
