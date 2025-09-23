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

Graph::Graph(Logger* log, MeasurementStorage* storage)
 : mLogger(log), mStorage(storage), mNeighborIndex(flann::KDTreeSingleIndexParams())
{
	// Initialize some members
	mSolver = NULL;	
	mFixNext = false;
	mOptimized = false;
	mConstraintsAdded = 0;
}

Graph::~Graph()
{
}

void Graph::setSolver(Solver* solver)
{
	if(mSolver)
	{
		delete mSolver;
	}
	mSolver = solver;
}

void Graph::reloadToSolver()
{
	if(!mSolver)
	{
		mLogger->message(ERROR, "A solver must be set before reloadToSolver() is called!");
		return;
	}

	// clear current solver
	mSolver->clear();

	// add all vertices 
	VertexObjectList vertices = getAllVertices();
	for (const auto& vertex : vertices)
	{
		mSolver->addVertex(vertex.index, vertex.correctedPose);
		if(vertex.fixed)
		{
			mSolver->setFixed(vertex.index);
		}
	}

	// add all edges after vertices are defined
	for (const auto& vertex : vertices)
	{
		for (const auto& edge : getOutEdges(vertex.index))
		{
			if (edge.constraint->getType() != TENTATIVE)
			{
				mSolver->addEdge(edge.source, edge.target, edge.constraint);
			}
		}
	}
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
	if(!mSolver->compute(iterations))
	{
		return false;
	}
	mOptimized = true;
	mConstraintsAdded = 0;

	// Retrieve results
	IdPoseVector res = mSolver->getCorrections();
	for(IdPoseVector::iterator it = res.begin(); it < res.end(); it++)
	{
		unsigned int id = it->first;
		Transform tf = it->second;
		try
		{
			setCorrectedPose(id, tf);
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
	VertexObject vo;
	vo.init(m, id);
	vo.correctedPose = corrected;
	vo.fixed = mFixNext;
	mFixNext = false;
	addVertex(vo);
	mStorage->add(m);
	mLogger->message(INFO, (boost::format("Created vertex %1% (from %2%:%3%).") % id % m->getRobotName() % m->getSensorName()).str());

	// Add it to the uuid-index, so we can find it by its uuid
	mUuidIndex.insert(UuidIndex::value_type(m->getUniqueId(), id));
	
	// Add it to the SLAM-Backend for incremental optimization
	if(mSolver)
	{
		mSolver->addVertex(id, corrected);
		if(vo.fixed)
		{
			mSolver->setFixed(id);
		}
	}
	return id;
}

void Graph::addTentativeConstraint(IdType source_id, IdType target_id, std::string& sensor)
{
	EdgeObject eo;
	eo.source = source_id;
	eo.target = target_id;
	eo.constraint = Constraint::Ptr(new TentativeConstraint(sensor));
	addEdge(eo);
}

void Graph::addConstraint(IdType source_id, IdType target_id, Constraint::Ptr c)
{
	// Create the new EdgeObject and add it to the PoseGraph
	EdgeObject eo;
	eo.source = source_id;
	eo.target = target_id;
	eo.constraint = c;
	addEdge(eo);
	mLogger->message(INFO, (boost::format("%3% created edge from node %1% to node %2% of type %4%.")
	 % eo.source % eo.target % eo.constraint->getSensorName() % eo.constraint->getTypeName()).str());
	
	// Add it to the SLAM-Backend for incremental optimization
	if(mSolver)
	{
		mSolver->addEdge(eo.source, eo.target, eo.constraint);
		mConstraintsAdded++;
	}
}

void Graph::removeConstraint(IdType source, IdType target, const std::string& sensor)
{
	// Remove from graph
	removeEdge(source, target, sensor);
	
	// Remove from solver
	// TODO
}

IdType Graph::getIndex(boost::uuids::uuid id) const
{
	return mUuidIndex.at(id);
}

bool Graph::hasMeasurement(boost::uuids::uuid id) const
{
	return mUuidIndex.find(id) != mUuidIndex.end();
}

const VertexObject Graph::getVertex(boost::uuids::uuid id) const
{
	return getVertex(mUuidIndex.at(id));
}

const Transform Graph::getTransform(IdType source, IdType target) const
{
	return getVertex(source).correctedPose.inverse() * getVertex(target).correctedPose;
}

const std::set<std::string> Graph::getVertexSensors() const
{
	std::set<std::string> sensors;
	for (const auto& vertex : getAllVertices())
	{
		sensors.insert(vertex.sensorName);
	}
	return sensors;
}

const std::set<std::string> Graph::getEdgeSensors() const
{
	std::set<std::string> sensors;
	for (const auto& edge : getEdges(getAllVertices()))
	{
		sensors.insert(edge.constraint->getSensorName());
	}
	return sensors;
}

Measurement::Ptr Graph::getMeasurement(IdType id)
{
	return mStorage->get(getVertex(id).measurementUuid);
}

Measurement::Ptr Graph::getMeasurement(boost::uuids::uuid id)
{
	return mStorage->get(id);
}

void Graph::buildNeighborIndex(const std::set<std::string>& sensors)
{
	VertexObjectList vertices;
	if(sensors.empty())
	{
		vertices = getAllVertices();
	}else
	{
		for(auto sensor : sensors)
		{
			VertexObjectList v = getVerticesFromSensor(sensor);
			vertices.insert(vertices.end(), v.begin(), v.end());
		}
	}

	int numOfVertices = vertices.size();
	if(numOfVertices == 0)
	{
		throw std::runtime_error("Cannot build neighbor index, vertex list is empty.");
	}
	flann::Matrix<float> points(new float[numOfVertices * 3], numOfVertices, 3);

	IdType row = 0;
	mNeighborMap.clear();
	for(VertexObjectList::iterator it = vertices.begin(); it < vertices.end(); ++it)
	{
		Transform::TranslationPart t = it->correctedPose.translation();
		points[row][0] = t[0];
		points[row][1] = t[1];
		points[row][2] = t[2];
		mNeighborMap.insert(std::map<IdType, IdType>::value_type(row, it->index));
		row++;
	}
	
	mNeighborIndex.buildIndex(points);
}

const VertexObjectList Graph::getNearbyVertices(const Transform &tf, float radius, const std::string& sensortype) const
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
	int found = mNeighborIndex.radiusSearch(query, neighbors, distances, radius, mSearchParams);
	
	// Write the result
	VertexObjectList result;
	std::vector<int>::iterator it = neighbors[0].begin();
	std::vector<NeighborIndex::DistanceType>::iterator d = distances[0].begin();
	if (sensortype == "") {
		for(; it < neighbors[0].end(); ++it, ++d) {	
			result.push_back(getVertex(mNeighborMap.at(*it)));
			mLogger->message(DEBUG, (boost::format(" - vertex %1% nearby (d = %2%)") % mNeighborMap.at(*it) % *d).str());
		}
	} else {
		for(; it < neighbors[0].end(); ++it, ++d) {	
			VertexObject vertex = getVertex(mNeighborMap.at(*it));
			if (vertex.typeName == sensortype) {
				result.push_back(vertex);
				mLogger->message(DEBUG, (boost::format(" - vertex %1% nearby (d = %2%)") % mNeighborMap.at(*it) % *d).str());
			}
		}
	}
	
	mLogger->message(DEBUG, (boost::format("Neighbor search found %1% vertices nearby.") % found).str());
	return result;
}