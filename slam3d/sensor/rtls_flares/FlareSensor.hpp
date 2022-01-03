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
		
		void setStatus(const std::string& json);
		
	protected:
		rtls_flares::Status mStatus;
		Clock mClock;
		Transform mCurrentPose;
		timeval mTimestamp;
		bool mHasNewData;
	};
}
