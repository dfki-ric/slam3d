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

#include "Graph.hpp"

#include <boost/format.hpp>

using namespace slam3d;

// Re-orthogonalize the rotation-matrix
Transform Graph::orthogonalize(const Transform& t)
{
	Eigen::Quaternion<ScalarType> q(t.linear());
	q.normalize();
	Transform res(t);
	res.linear() = q.toRotationMatrix();
	return res;
}

Graph::Graph(Logger* log)
 : mLogger(log), mNeighborIndex(flann::KDTreeSingleIndexParams())
{
	// Initialize some members
	mSolver = NULL;	
	mOptimized = false;
	mLastIndex = 0;
}

Graph::~Graph()
{
}

void Graph::setSolver(Solver* solver)
{
	mSolver = solver;
//	mSolver->addNode(0, Transform::Identity());
//	mSolver->setFixed(0);
}

void Graph::registerPoseSensor(PoseSensor* s)
{
	std::pair<PoseSensorList::iterator, bool> result;
	result = mPoseSensors.insert(PoseSensorList::value_type(s->getName(), s));
	if(!result.second)
	{
		mLogger->message(ERROR, (boost::format("PoseSensor with name %1% already exists!") % s->getName()).str());
		return;
	}
}

void Graph::registerSensor(Sensor* s)
{
	std::pair<SensorList::iterator, bool> result;
	result = mSensors.insert(SensorList::value_type(s->getName(), s));
	if(!result.second)
	{
		mLogger->message(ERROR, (boost::format("Sensor with name %1% already exists!") % s->getName()).str());
		return;
	}
	s->setGraph(this);
}

Transform Graph::getCurrentPose()
{
	return getVertex(mLastIndex).corrected_pose;
}

void Graph::writeGraphToFile(const std::string &name)
{
	mLogger->message(ERROR, "Graph writing not implemented!");
}

bool Graph::hasSensorForMeasurement(Measurement::Ptr measurement)
{
	return mSensors.find(measurement->getSensorName()) != mSensors.end();
}

bool Graph::getSensorForMeasurement(Measurement::Ptr measurement, Sensor*& sensor)
{
	SensorList::iterator it = mSensors.find(measurement->getSensorName());
	if(it != mSensors.end())
	{
		sensor = it->second;
		return true;
	}
	return false;
}

bool Graph::optimized()
{
	if(mOptimized)
	{
		mOptimized = false;
		return true;
	}else
	{
		return false;
	}
}

IdType Graph::addVertex(Measurement::Ptr m, const Transform &corrected)
{
	// Create the new VertexObject and add it to the PoseGraph
	IdType id = mIndexer.getNext();
	boost::format v_name("%1%:%2%(%3%)");
	v_name % m->getRobotName() % m->getSensorName() % id;
	VertexObject vo;
	vo.index = id;
	vo.label = v_name.str();
	vo.corrected_pose = corrected;
	vo.measurement = m;
	addVertex(vo);

	// Add it to the uuid-index, so we can find it by its uuid
	mUuidIndex.insert(UuidIndex::value_type(m->getUniqueId(), id));
	
	// Add it to the SLAM-Backend for incremental optimization
	if(mSolver)
	{
		mSolver->addNode(id, corrected);
		if(id == 0)
			mSolver->setFixed(0);
	}
	mLogger->message(INFO, (boost::format("Created vertex %1% (from %2%:%3%).") % id % m->getRobotName() % m->getSensorName()).str());
	return id;
}

IdType Graph::addMeasurement(Measurement::Ptr m)
{
	// Add root node to the graph
	if(mLastIndex == 0)
	{
		mLogger->message(DEBUG, "Adding map origin before first node.");
		mLastIndex = addVertex(Measurement::Ptr(new MapOrigin()), Transform::Identity());
	}
	
	// Add the vertex to the pose graph
	mLogger->message(DEBUG, (boost::format("Add reading from own Sensor '%1%'.") % m->getSensorName()).str());
	mLastIndex = addVertex(m, getCurrentPose());
	
	// Link first node to root
	if(mLastIndex == 1)
	{
		addConstraint(0, 1, Transform::Identity(), Covariance::Identity(), "none", "root-link");
	}
	
	// Call all registered PoseSensor's on the new vertex
	for(PoseSensorList::iterator ps = mPoseSensors.begin(); ps != mPoseSensors.end(); ps++)
	{
		ps->second->handleNewVertex(mLastIndex);
	}
	
	// Return the new vertex index
	return mLastIndex;
}

void Graph::addExternalMeasurement(Measurement::Ptr m, boost::uuids::uuid s, const Transform& tf, const Covariance& cov, const std::string& sensor)
{
	if(hasMeasurement(m->getUniqueId()))
	{
		throw DuplicateMeasurement();
	}
	
	Transform pose = getVertex(s).corrected_pose * tf;
	IdType source = mUuidIndex.at(s);
	IdType target = addVertex(m, pose);
	addConstraint(source, target, tf, cov, sensor, "ext");
}

void Graph::addExternalConstraint(boost::uuids::uuid s, boost::uuids::uuid t, const Transform& tf, const Covariance& cov, const std::string& sensor)
{
	IdType source = mUuidIndex.at(s);
	IdType target = mUuidIndex.at(t);

	try
	{
		getEdge(source, target, sensor);
		throw DuplicateEdge(source, target, sensor);
	}catch(InvalidEdge &ie)
	{
		addConstraint(source, target, tf, cov, sensor, "ext");
	}
}

bool Graph::hasMeasurement(boost::uuids::uuid id) const
{
	return mUuidIndex.find(id) != mUuidIndex.end();
}

const VertexObject& Graph::getVertex(boost::uuids::uuid id) const
{
	return getVertex(mUuidIndex.at(id));
}

TransformWithCovariance Graph::getTransform(IdType source, IdType target) const
{
	// This method is a stub:
	// Replace with something more elaborate, that calculates the covariance as well.
	TransformWithCovariance twc;
	twc.transform = getVertex(source).corrected_pose.inverse() * getVertex(target).corrected_pose;
	return twc;
}

void Graph::buildNeighborIndex(const std::string& sensor)
{
	VertexObjectList vertices = getVerticesFromSensor(sensor);
	int numOfVertices = vertices.size();
	flann::Matrix<float> points(new float[numOfVertices * 3], numOfVertices, 3);

	IdType row = 0;
	mNeighborMap.clear();
	for(VertexObjectList::iterator it = vertices.begin(); it < vertices.end(); ++it)
	{
		Transform::TranslationPart t = it->corrected_pose.translation();
		points[row][0] = t[0];
		points[row][1] = t[1];
		points[row][2] = t[2];
		mNeighborMap.insert(std::map<IdType, IdType>::value_type(row, it->index));
		row++;
	}
	
	mNeighborIndex.buildIndex(points);
}

VertexObjectList Graph::getNearbyVertices(const Transform &tf, float radius) const
{
	// Fill in the query point
	flann::Matrix<float> query(new float[3], 1, 3);
	Transform::ConstTranslationPart t = tf.translation();
	query[0][0] = t[0];
	query[0][1] = t[1];
	query[0][2] = t[2];
	mLogger->message(DEBUG, (boost::format("Doing NN search from (%1%, %2%, %3%) with radius %4%.")%t[0]%t[1]%t[2]%radius).str());
	
	// Find points nearby
	std::vector< std::vector<int> > neighbors;
	std::vector< std::vector<NeighborIndex::DistanceType> > distances;
	int found = mNeighborIndex.radiusSearch(query, neighbors, distances, radius*radius, mSearchParams);
	
	// Write the result
	VertexObjectList result;
	std::vector<int>::iterator it = neighbors[0].begin();
	std::vector<NeighborIndex::DistanceType>::iterator d = distances[0].begin();
	for(; it < neighbors[0].end(); ++it, ++d)
	{
		result.push_back(getVertex(mNeighborMap.at(*it)));
		mLogger->message(DEBUG, (boost::format(" - vertex %1% nearby (d = %2%)") % mNeighborMap.at(*it) % *d).str());
	}
	
	mLogger->message(DEBUG, (boost::format("Neighbor search found %1% vertices nearby.") % found).str());
	return result;
}
