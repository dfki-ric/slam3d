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
	mConstraintsAdded = 0;
	mOptimizationRate = 0;
}

Graph::~Graph()
{
}

void Graph::setSolver(Solver* solver, unsigned rate)
{
	mSolver = solver;
	mOptimizationRate = rate;
}

void Graph::writeGraphToFile(const std::string &name)
{
	mLogger->message(ERROR, "Graph writing not implemented!");
}

bool Graph::optimize(unsigned iterations)
{
	if(!mSolver)
	{
		mLogger->message(ERROR, "A solver must be set before optimize() is called!");
		return false;
	}

	// Optimize
	if(!mSolver->compute())
	{
		return false;
	}
	mOptimized = true;

	// Retrieve results
	IdPoseVector res = mSolver->getCorrections();
	for(IdPoseVector::iterator it = res.begin(); it < res.end(); it++)
	{
		unsigned int id = it->first;
		Transform tf = it->second;
		try
		{
			getVertexInternal(id).corrected_pose = tf;
		}catch(std::out_of_range &e)
		{
			mLogger->message(ERROR, (boost::format("Vertex with id %1% does not exist!") % id).str());
		}
	}
	return true;
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
	mLogger->message(INFO, (boost::format("Created vertex %1% (from %2%:%3%).") % id % m->getRobotName() % m->getSensorName()).str());

	// Add it to the uuid-index, so we can find it by its uuid
	mUuidIndex.insert(UuidIndex::value_type(m->getUniqueId(), id));
	
	// Add it to the SLAM-Backend for incremental optimization
	if(mSolver)
	{
		mSolver->addVertex(id, corrected);
		if(id == 0)
			mSolver->setFixed(0);
	}
	return id;
}

void Graph::addConstraint(IdType source_id, IdType target_id, Constraint::Ptr c)
{
	// Create the new EdgeObject and add it to the PoseGraph
	EdgeObject eo;
	eo.source = source_id;
	eo.target = target_id;
	eo.constraint = c;
	addEdge(eo);
	mConstraintsAdded++;
	mLogger->message(INFO, (boost::format("%3% created edge from node %1% to node %2% of type %4%.")
	 % source_id % target_id % c->getSensorName() % c->getTypeName()).str());
	
	// Add it to the SLAM-Backend for incremental optimization
	if(mSolver)
	{
		mSolver->addEdge(source_id, target_id, c);
		if(mOptimizationRate > 0 && (mConstraintsAdded % mOptimizationRate) == 0)
			optimize();
	}
}

IdType Graph::getIndex(boost::uuids::uuid id) const
{
	return mUuidIndex.at(id);
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
	if(numOfVertices == 0)
	{
		throw std::runtime_error((boost::format("Cannot build neighbor index, because there are no vertices from %1%.") % sensor).str());
	}
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

void Graph::setCorrectedPose(IdType id, const Transform& pose)
{
	getVertexInternal(id).corrected_pose = pose;
}
