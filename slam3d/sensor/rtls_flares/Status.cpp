#include "Status.hpp"

#include <sstream>

using namespace rtls_flares;

Position::Position(const Json::Value& json)
{
	x = json["x"].asDouble();
	y = json["y"].asDouble();
	z = json["z"].asDouble();
}

Orientation::Orientation(const Json::Value& json)
{
	heading = json["h"].asDouble();
	roll = json["r"].asDouble();
	pitch = json["p"].asDouble();
}

Tag::Tag(const Json::Value& json)
 : currentPosition(json["pos"])
{
	UDID = json["id16"].asString();
	timestamp = json["ts"].asUInt();
	DoF = json["dof"].asUInt();
}

Anchor::Anchor(const Json::Value& json)
 : Tag(json)
{
	currentDistance=json["ldis"].asDouble();
	averageDistance=json["adis"].asDouble();
	varianceOfDistance=json["var"].asDouble();
	numberOfMeasurements=json["nom"].asUInt();
	sequenceNumber=json["sqn"].asUInt();
//	firstPathPower=json["ra"].asDouble();
//	receivedPower=json["receivePower"].asDouble();
	channelMaximumNoise=json["blk"].asDouble();
	timeoutCounter=json["toc"].asUInt();
}

Status::Status(const Json::Value& json)
 : mIMU(json["imu"]), mCurrentPosition(json["pos"]), mMeanPosition(json["mean"])
{
	mTimestamp = json["t"].asUInt();
	mUDID = json["id16"].asString();
	
	mNumberOfMeasurements=json["nom"].asUInt();
	mNumberOfKnownAnchors=json["noga"].asUInt();
	mNumberOfUsedAnchors=json["nora"].asUInt();

	// Anchors and Tags
	for(auto& it : json["al"])
	{
		mAnchorList.push_back(Anchor(it));
	}
	for(auto& it : json["tl"])
	{
		mTagList.push_back(Tag(it));
	}
}

Json::Value toJSON(const std::string& input)
{
	Json::Value inputJson;
	std::stringstream inputStream(input);
	inputStream >> inputJson;
	return inputJson;
}

Status::Status(const std::string& str)
: Status(toJSON(str)) {}

const std::string emptyStatus = "{\"t\":0,\"id16\":\"0x0\",\"pos\":{\"x\":0,\"y\":0,\"z\":0},\"mean\":{\"x\":0,\"y\":0,\"z\":0},\"imu\":{\"h\":0,\"r\":0,\"p\":0},\"nom\":0,\"pr\":1.0000,\"noga\":0,\"nora\":0,\"dof\":0,\"Status\":{\"md\":0,\"txR\":0,\"rxR\":0},\"m\":\"ft\",\"al\":[],\"tl\":[]}";

Status::Status()
: Status(emptyStatus) {}
