// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2017 S. Kasperski
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "Mapper.hpp"

#include <boost/format.hpp>

using namespace slam3d;

Mapper::Mapper(Graph* graph, Logger* log)
{
	mGraph = graph;
	mLogger = log;
	mLastIndex = 0;
}

Mapper::~Mapper()
{
	
}

void Mapper::registerPoseSensor(PoseSensor* s)
{
	std::pair<PoseSensorList::iterator, bool> result;
	result = mPoseSensors.insert(PoseSensorList::value_type(s->getName(), s));
	if(!result.second)
	{
		mLogger->message(ERROR, (boost::format("PoseSensor with name %1% already exists!") % s->getName()).str());
		return;
	}
}

void Mapper::registerSensor(Sensor* s)
{
	std::pair<SensorList::iterator, bool> result;
	result = mSensors.insert(SensorList::value_type(s->getName(), s));
	if(!result.second)
	{
		mLogger->message(ERROR, (boost::format("Sensor with name %1% already exists!") % s->getName()).str());
		return;
	}
	s->setMapperAndGraph(this, mGraph);
}

Transform Mapper::getCurrentPose()
{
	return mGraph->getVertex(mLastIndex).corrected_pose;
}

IdType Mapper::addMeasurement(Measurement::Ptr m)
{
	// Add root node to the graph
	if(mLastIndex == 0)
	{
		mLogger->message(DEBUG, "Adding map origin before first node.");
		mLastIndex = mGraph->addVertex(Measurement::Ptr(new MapOrigin()), Transform::Identity());
	}
	
	// Add the vertex to the pose graph
	mLogger->message(DEBUG, (boost::format("Add reading from own Sensor '%1%'.") % m->getSensorName()).str());
	mLastIndex = mGraph->addVertex(m, getCurrentPose());
	
	// Link first node to root
	if(mLastIndex == 1)
	{
		SE3Constraint::Ptr se3(new SE3Constraint("root-link", TransformWithCovariance()));
		mGraph->addConstraint(0, 1, se3);
	}
	
	// Call all registered PoseSensor's on the new vertex
	for(PoseSensorList::iterator ps = mPoseSensors.begin(); ps != mPoseSensors.end(); ps++)
	{
		ps->second->handleNewVertex(mLastIndex);
	}
	
	// Return the new vertex index
	return mLastIndex;
}

void Mapper::addExternalMeasurement(Measurement::Ptr m, boost::uuids::uuid s, const TransformWithCovariance& twc, const std::string& sensor)
{
	if(mGraph->hasMeasurement(m->getUniqueId()))
	{
		throw DuplicateMeasurement();
	}
	
	Transform pose = mGraph->getVertex(s).corrected_pose * twc.transform;
	IdType source = mGraph->getIndex(s);
	IdType target = mGraph->addVertex(m, pose);
	SE3Constraint::Ptr se3(new SE3Constraint(sensor, twc));
	mGraph->addConstraint(source, target, se3);
}

void Mapper::addExternalConstraint(boost::uuids::uuid s, boost::uuids::uuid t, const TransformWithCovariance& twc, const std::string& sensor)
{
	IdType source = mGraph->getIndex(s);
	IdType target = mGraph->getIndex(t);

	try
	{
		mGraph->getEdge(source, target, sensor);
		throw DuplicateEdge(source, target, sensor);
	}catch(InvalidEdge &ie)
	{
		SE3Constraint::Ptr se3(new SE3Constraint(sensor, twc));
		mGraph->addConstraint(source, target, se3);
	}
}

const VertexObject& Mapper::getLastVertex() const
{
	return mGraph->getVertex(mLastIndex);
}