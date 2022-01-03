#pragma once
#include <string>
#include <vector>
#include <jsoncpp/json/json.h>

namespace rtls_flares
{
	struct Position
	{
		Position(const Json::Value& json);
		
		double x = 0;
		double y = 0;
		double z = 0;
	};
	
	struct Orientation
	{
		Orientation(const Json::Value& json);
		
		double heading = 0;
		double roll = 0;
		double pitch = 0;
	};

	// A tag is any beacon in the ad-hoc rtls network
	struct Tag
	{
		Tag(const Json::Value& json);

		std::string UDID;
		unsigned timestamp;
		Position currentPosition;
		unsigned char DoF;
	};

	// An anchor is a tag which has successfully localized itself and is now
	// considered to be static.
	struct Anchor : public Tag
	{
		Anchor(const Json::Value& json);
		
		double positionRate;
		double currentDistance;
		double averageDistance;
		double varianceOfDistance;
		unsigned numberOfMeasurements;
		unsigned sequenceNumber;
		double firstPathPower;
		double receivedPower;
		double channelStandardNoise; 
		double channelMaximumNoise;
		unsigned timeoutCounter;
		bool active; 
	};

	struct Status
	{
		Status(const Json::Value& json);
		Status(const std::string& str);
		Status();
		
		unsigned mTimestamp;
		std::string mUDID;
		Position mCurrentPosition;
		Position mMeanPosition;
		Orientation mIMU;
		
		unsigned mNumberOfMeasurements;
		unsigned mNumberOfKnownAnchors;
		unsigned mNumberOfUsedAnchors;
		
		std::vector<Anchor>  mAnchorList;
		std::vector<Tag> mTagList;
	};
}
