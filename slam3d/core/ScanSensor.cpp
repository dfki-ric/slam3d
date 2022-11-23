// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2019 S. Kasperski
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

#include "ScanSensor.hpp"
#include "Mapper.hpp"

#include <boost/format.hpp>
#include <boost/thread.hpp>

using namespace slam3d;

ScanSensor::ScanSensor(const std::string& n, Logger* l)
 : Sensor(n,l), mPatchSolver(NULL)
{
	mNeighborRadius = 1.0;
	mMaxNeighorLinks = 1;
	mMinLoopLength = 10;
	mLinkPrevious = true;
	mLastTransform = Transform::Identity();
	mLinkSensors.insert(n);
}

ScanSensor::~ScanSensor()
{
}

bool ScanSensor::addMeasurement(const Measurement::Ptr& m)
{
	if(mLastVertex == 0)
	{
		mLastVertex = mMapper->addMeasurement(m);
		return true;
	}

	Measurement::Ptr source = mMapper->getGraph()->getVertex(mLastVertex).measurement;
	try
	{
		Constraint::Ptr c = createConstraint(source, m, mLastTransform, false);
		SE3Constraint::Ptr se3 = boost::dynamic_pointer_cast<SE3Constraint>(c);
		if(!se3 || checkMinDistance(mLastTransform = se3->getRelativePose().transform))
		{
			IdType newVertex = mMapper->addMeasurement(m);
			if(se3)
			{
				Transform pose = mMapper->getCurrentPose() * mLastTransform;
				mMapper->getGraph()->setCorrectedPose(newVertex, pose);
				mLastTransform = Transform::Identity();
			}
			mMapper->getGraph()->addConstraint(mLastVertex, newVertex, c);
			mLastVertex = newVertex;
			return true;
		}
	}catch(std::exception &e)
	{
		mLogger->message(WARNING, (boost::format("Could not add Measurement: %1%") % e.what()).str());
	}
	return false;
}

bool ScanSensor::addMeasurement(const Measurement::Ptr& m, const Transform& odom)
{
	if(mLastVertex == 0)
	{
		mLastVertex = mMapper->addMeasurement(m);
		mLastOdometry = odom;
		return true;
	}
	
	// Add measurement if sufficient movement is reported by the odometry
	Transform odom_delta = mLastOdometry.inverse() * odom;
	if(checkMinDistance(odom_delta))
	{
		IdType newVertex = mMapper->addMeasurement(m);
		Measurement::Ptr source = mMapper->getGraph()->getVertex(mLastVertex).measurement;
		if(mLinkPrevious)
		{
			try
			{
				Constraint::Ptr c = createConstraint(source, m, odom_delta, false);
				mMapper->getGraph()->addConstraint(mLastVertex, newVertex, c);

				// Calculate the new pose relative from last pose
				SE3Constraint::Ptr se3 = boost::dynamic_pointer_cast<SE3Constraint>(c);
				if(se3)
				{
					const Transform& lastPose = mMapper->getGraph()->getVertex(mLastVertex).corrected_pose;
					mMapper->getGraph()->setCorrectedPose(newVertex, lastPose * se3->getRelativePose().transform);
				}
			}catch(std::exception &e)
			{
				mLogger->message(WARNING, (boost::format("Could not link Measurement to previous: %1%") % e.what()).str());
			}
		}
		mLastOdometry = odom;
		mLastVertex = newVertex;
		return true;
	}
	return false;
}

void ScanSensor::link(IdType source_id, IdType target_id)
{
	// We have no guess, so we use the current relative pose from the graph
	Transform guess = mMapper->getGraph()->getTransform(source_id, target_id).transform;
	link(source_id, target_id, guess);
}

void ScanSensor::link(IdType source_id, IdType target_id, const Transform& guess)
{
	// Add a placeholder before starting the computation
	mMapper->getGraph()->addTentativeConstraint(source_id, target_id, mName);
	
	// Build local patches around source and target
	Measurement::Ptr source_m = buildPatch(source_id);
	Measurement::Ptr target_m = buildPatch(target_id);

	// Create the relative pose constraint
	try
	{
		Constraint::Ptr se3 = createConstraint(source_m, target_m, guess, true);
		mMapper->getGraph()->replaceConstraint(source_id, target_id, se3);
	}catch(NoMatch &e)
	{
		mLogger->message(WARNING, (boost::format("Failed to link vertex %1% and %2%, because %3%.") % source_id % target_id % e.what()).str());
		// delete tentative constraint
		return;
	}
	
}

void ScanSensor::linkToNeighbors(IdType vertex)
{
	if(mMaxNeighorLinks == 0)
		return;

	mMapper->getGraph()->buildNeighborIndex(mLinkSensors);
	VertexObject obj = mMapper->getGraph()->getVertex(vertex);
	VertexObjectList neighbors = mMapper->getGraph()->getNearbyVertices(obj.corrected_pose, mNeighborRadius);
	
	int count = 0;
	for(auto i = neighbors.rbegin(); i != neighbors.rend() && count < mMaxNeighorLinks; i++)
	{
		IdType index = i->index;
		if(index == vertex) continue;
		try
		{
			mMapper->getGraph()->getEdge(vertex, index, mName);
			continue;
		}
		catch(InvalidEdge &e){}
		catch(InvalidVertex &e)
		{
			mLogger->message(ERROR, e.what());
			return;
		}

		float dist = mMapper->getGraph()->calculateGraphDistance(index, vertex);
		mLogger->message(DEBUG, (boost::format("Distance(%2%,%3%) in Graph is: %1%") % dist % index % vertex).str());
		if(dist <= mPatchBuildingRange * 2 || dist < mMinLoopLength)
			continue;
		count++;
		link(index, vertex);
	}
}

void ScanSensor::linkLastToNeighbors(bool mt)
{
	if(mMaxNeighorLinks < 1)
		return;

	if(mt)
		boost::thread linkThread(&ScanSensor::linkToNeighbors, this, mLastVertex);
	else
		linkToNeighbors(mLastVertex);
}

Measurement::Ptr ScanSensor::buildPatch(IdType source)
{
	if(mPatchBuildingRange == 0)
	{
		return mMapper->getGraph()->getVertex(source).measurement;
	}

	VertexObjectList v_objects = mMapper->getGraph()->getVerticesInRange(source, mPatchBuildingRange);
	mLogger->message(DEBUG, (boost::format("Building pointcloud patch from %1% nodes.") % v_objects.size()).str());
	
	if(mPatchSolver)
	{
		std::lock_guard<std::mutex> guard(mPatchSolverMutex);
		mPatchSolver->clear();
		for(VertexObjectList::iterator v = v_objects.begin(); v < v_objects.end(); v++)
		{
			mPatchSolver->addVertex(v->index, v->corrected_pose);
		}
		
		EdgeObjectList e_objects = mMapper->getGraph()->getEdges(v_objects);
		for(EdgeObjectList::iterator e = e_objects.begin(); e < e_objects.end(); e++)
		{
			if(e->constraint->getType() != SE3)
				continue;
			try
			{
				mPatchSolver->addEdge(e->source, e->target, e->constraint);
			}catch(Solver::BadEdge &be)
			{
				mLogger->message(ERROR, be.what());
			}
		}
		
		mPatchSolver->setFixed(source);
		mPatchSolver->compute();
		IdPoseVector res = mPatchSolver->getCorrections();
		for(IdPoseVector::iterator it = res.begin(); it < res.end(); it++)
		{
			bool ok = false;
			for(VertexObjectList::iterator v = v_objects.begin(); v < v_objects.end(); v++)
			{
				if(v->index == it->first)
				{
					v->corrected_pose = it->second;
					ok = true;
					break;
				}
			}
			if(!ok)
			{
				mLogger->message(ERROR, (boost::format("Could not apply patch-solver result for vertex %1%!") % it->first).str());
			}
		}
	}
	return createCombinedMeasurement(v_objects, mMapper->getGraph()->getVertex(source).corrected_pose);
}

void ScanSensor::setNeighborRadius(float r, unsigned l)
{
	mLogger->message(INFO, (boost::format("neighbor_radius:        %1%") % r).str());
	mLogger->message(INFO, (boost::format("max_neighbor_links:     %1%") % l).str());
	mNeighborRadius = r;
	mMaxNeighorLinks = l;
}

void ScanSensor::setMinLoopLength(unsigned l)
{
	mLogger->message(INFO, (boost::format("min_loop_length:        %1%") % l).str());
	mMinLoopLength = l;
}

void ScanSensor::setLinkPrevious(bool l)
{
	mLogger->message(INFO, (boost::format("link_previous:          %1%") % l).str());
	mLinkPrevious = l;
}

void ScanSensor::setPatchBuildingRange(unsigned r)
{
	mLogger->message(INFO, (boost::format("patch_building_range:   %1%") % r).str());
	mPatchBuildingRange = r;
}

Transform ScanSensor::getCurrentPose() const
{
	return mMapper->getCurrentPose() * mLastTransform;
}