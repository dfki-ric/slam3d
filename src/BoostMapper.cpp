#include "BoostMapper.hpp"
#include "Solver.hpp"

#include <boost/format.hpp>

using namespace slam3d;

BoostMapper::BoostMapper(Logger* log)
 : GraphMapper(log), mNeighborIndex(flann::KDTreeSingleIndexParams())
{
	mLastVertex = 0;
	mFirstVertex = 0;
}

BoostMapper::~BoostMapper()
{
	
}

VertexList BoostMapper::getVerticesFromSensor(const std::string& sensor)
{
	VertexList vertexList;
	VertexRange vertices = boost::vertices(mPoseGraph);
	for(VertexIterator it = vertices.first; it != vertices.second; ++it)
	{
		if(mPoseGraph[*it].measurement->getSensorName() == sensor)
		{
			vertexList.push_back(*it);
		}
	}
	return vertexList;
}

EdgeList BoostMapper::getEdgesFromSensor(const std::string& sensor)
{
	EdgeList edgeList;
	EdgeRange edges = boost::edges(mPoseGraph);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		edgeList.push_back(*it);
	}
	return edgeList;
}

EdgeObjectList BoostMapper::getEdgeObjectsFromSensor(const std::string& sensor)
{
	EdgeObjectList objectList;
	EdgeRange edges = boost::edges(mPoseGraph);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		objectList.push_back(mPoseGraph[*it]);
	}
	return objectList;
}

void BoostMapper::buildNeighborIndex(const std::string& sensor)
{
	VertexList vertices = getVerticesFromSensor(sensor);
	int numOfVertices = vertices.size();
	flann::Matrix<float> points(new float[numOfVertices * 3], numOfVertices, 3);

	IdType row = 0;
	mNeighborMap.clear();
	for(VertexList::iterator it = vertices.begin(); it < vertices.end(); ++it)
	{
		Transform::TranslationPart t = mPoseGraph[*it].corrected_pose.translation();
		points[row][0] = t[0];
		points[row][1] = t[1];
		points[row][2] = t[2];
		mNeighborMap.insert(IndexMap::value_type(row, *it));
		row++;
	}
	
	mNeighborIndex.buildIndex(points);
}

VertexList BoostMapper::getNearbyVertices(const Transform &tf, float radius)
{
	// Fill in the query point
	flann::Matrix<float> query(new float[3], 1, 3);
	Transform::ConstTranslationPart t = tf.translation();
	query[0][0] = t[0];
	query[0][1] = t[1];
	query[0][2] = t[2];
	
	// Find points nearby
	std::vector< std::vector<int> > neighbors;
	std::vector< std::vector<NeighborIndex::DistanceType> > distances;
	mNeighborIndex.radiusSearch(query, neighbors, distances, radius, mSearchParams);
	
	// Write the result
	VertexList result;
	std::vector<int>::iterator it;
	for(it = neighbors[0].begin(); it < neighbors[0].end(); it++)
	{
		result.push_back(mNeighborMap[*it]);
	}
	
	return result;
}

bool BoostMapper::optimize()
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

	// Retrieve results
	IdPoseVector res = mSolver->getCorrections();
	for(IdPoseVector::iterator it = res.begin(); it < res.end(); it++)
	{
		unsigned int id = it->first;
		Transform tf = it->second;
		Vertex v = mNeighborMap[id];
		mPoseGraph[v].corrected_pose = tf;
	}
	
	if(mLastVertex)
	{
		mCurrentPose = mPoseGraph[mLastVertex].corrected_pose;
	}
	return true;
}

bool BoostMapper::addReading(Measurement* m)
{
	// Get the sensor responsible for this measurement
	// Can throw std::out_of_range if sensor is not registered
	Sensor* sensor = NULL;
	try
	{
		sensor = mSensors.at(m->getSensorName());
		mLogger->message(DEBUG, (boost::format("Add reading from own Sensor '%1%'.") % m->getSensorName()).str());
	}catch(std::out_of_range e)
	{
		mLogger->message(ERROR, (boost::format("Sensor '%1%' has not been registered!") % m->getSensorName()).str());
		return false;
	}

	// Get the odometric pose for this measurement
	Transform odometry = Transform::Identity();
	if(mOdometry)
	{
		try
		{
			odometry = mOdometry->getOdometricPose(m->getTimestamp());
		}catch(OdometryException &e)
		{
			mLogger->message(ERROR, "Could not get Odometry data!");
			return false;
		}
	}

	// If this is the first vertex, add it and return
	if(!mLastVertex)
	{
		mLastVertex = addVertex(m, mCurrentPose);
		mLastOdometricPose = odometry;
		mLogger->message(INFO, "Added first node to the graph.");
		return true;
	}

	// Now we have a node, that is not the first and has not been added yet
	Vertex newVertex = 0;
	
	if(mOdometry)
	{
		Transform odom_dist = orthogonalize(mLastOdometricPose.inverse() * odometry);
		mCurrentPose = mPoseGraph[mLastVertex].corrected_pose * odom_dist;
		if(!checkMinDistance(odom_dist))
			return false;

		if(mAddOdometryEdges)
		{
			// Add the vertex to the pose graph
			newVertex = addVertex(m, orthogonalize(mCurrentPose));

			// Add an edge representing the odometry information
			addEdge(mLastVertex, newVertex, odom_dist, Covariance::Identity(), "Odometry", "odom");
		}
	}

	// Add edge to previous measurement
	if(mLastVertex)
	{
		try
		{
			Transform lastPose = mPoseGraph[mLastVertex].corrected_pose;
			Transform guess = lastPose.inverse() * mCurrentPose;
			TransformWithCovariance twc = sensor->calculateTransform(mPoseGraph[mLastVertex].measurement, m, guess);
			mCurrentPose = orthogonalize(lastPose * twc.transform);
			
			if(newVertex)
			{
				mPoseGraph[newVertex].corrected_pose = mCurrentPose;
			}else
			{
				if(!checkMinDistance(twc.transform))
					return false;
				newVertex = addVertex(m, mCurrentPose);
			}
			addEdge(mLastVertex, newVertex, twc.transform, twc.covariance, sensor->getName(), "seq");
		}catch(NoMatch &e)
		{
			if(!newVertex)
			{
				mLogger->message(WARNING, "Measurement could not be matched and no odometry was availabe!");
				return false;
			}
		}
	}

	// Add edges to other measurements nearby
	buildNeighborIndex(sensor->getName());
	linkToNeighbors(newVertex, sensor, mMaxNeighorLinks);

	// Overall last vertex
	mLastVertex = newVertex;
	mLastOdometricPose = odometry;
	return true;
}

Vertex BoostMapper::addVertex(Measurement* m, const Transform &corrected)
{
	// Create the new VertexObject and add it to the PoseGraph
	boost::format v_name("%1%:%2%");
	v_name % m->getRobotName() % m->getSensorName();
	IdType id = mIndexer.getNext();
	Vertex newVertex = boost::add_vertex(mPoseGraph);
	mPoseGraph[newVertex].index = id;
	mPoseGraph[newVertex].label = v_name.str();
	mPoseGraph[newVertex].corrected_pose = corrected;
	mPoseGraph[newVertex].measurement = m;

	// Add it to the indexes, so we can find it by its id and uuid
	mIndexMap.insert(IndexMap::value_type(id, newVertex));
	mVertexIndex.insert(UuidMap::value_type(m->getUniqueId(), newVertex));
	
	// Add it to the SLAM-Backend for incremental optimization
	if(mSolver)
	{
		mSolver->addNode(id, corrected);
	}
	
	// Set it as fixed in the solver
	if(!mFirstVertex)
	{
		mFirstVertex = newVertex;
		if(mSolver)
		{
			mSolver->setFixed(id);
		}
	}
	
	mLogger->message(INFO, (boost::format("Created vertex %1% (from %2%:%3%).") % id % m->getRobotName() % m->getSensorName()).str());
	return newVertex;
}

Edge BoostMapper::addEdge(Vertex source, Vertex target,
	const Transform &t, const Covariance &c, const std::string& sensor, const std::string& label)
{
	Edge edge;
	bool inserted;
	boost::tie(edge, inserted) = boost::add_edge(source, target, mPoseGraph);
	unsigned source_id = mPoseGraph[source].index;
	unsigned target_id = mPoseGraph[target].index;

	mPoseGraph[edge].transform = t;
	mPoseGraph[edge].covariance = c;
	mPoseGraph[edge].sensor = sensor;
	mPoseGraph[edge].label = label;
	mPoseGraph[edge].source = source_id;
	mPoseGraph[edge].target = target_id;
	
	if(mSolver)
	{
		mSolver->addConstraint(source_id, target_id, t, c);
	}
	mLogger->message(INFO, (boost::format("Created '%4%' edge from node %1% to node %2% (from %3%).") % source_id % target_id % sensor % label).str());
	return edge;
}


void BoostMapper::addExternalReading(Measurement* m, const Transform& t)
{
	Vertex v = addVertex(m, t);
		
	// Get the sensor responsible for this measurement
	// Can throw std::out_of_range if sensor is not registered
	mLogger->message(DEBUG, (boost::format("Add external reading from %1%:%2%.") % m->getRobotName() % m->getSensorName()).str());
	Sensor* sensor = NULL;
	try
	{
		sensor = mSensors.at(m->getSensorName());
		buildNeighborIndex(sensor->getName());
		linkToNeighbors(v, sensor, mMaxNeighorLinks);
	}catch(std::out_of_range e)
	{
		return;
	}
}

void BoostMapper::linkToNeighbors(Vertex vertex, Sensor* sensor, int max_links)
{
	// Get all edges to/from this node
	std::set<Vertex> previously_matched_vertices;
	previously_matched_vertices.insert(vertex);
	
	OutEdgeIterator out_it, out_end;
	boost::tie(out_it, out_end) = boost::out_edges(vertex, mPoseGraph);
	for(; out_it != out_end; ++out_it)
	{
		if(mPoseGraph[*out_it].sensor == sensor->getName())
		{
			previously_matched_vertices.insert(boost::target(*out_it, mPoseGraph));
		}
	}
	
	InEdgeIterator in_it, in_end;
	boost::tie(in_it, in_end) = boost::in_edges(vertex, mPoseGraph);
	for(; in_it != in_end; ++in_it)
	{
		if(mPoseGraph[*in_it].sensor == sensor->getName())
		{
			previously_matched_vertices.insert(boost::source(*in_it, mPoseGraph));
		}
	}
	
	std::vector<Vertex> neighbors = getNearbyVertices(mPoseGraph[vertex].corrected_pose, mNeighborRadius);
	mLogger->message(DEBUG, (boost::format("Neighbor search found %1% vertices nearby.") % neighbors.size()).str());
	
	int added = 0;
	for(std::vector<Vertex>::iterator it = neighbors.begin(); it != neighbors.end() && added < max_links; ++it)
	{
		if(previously_matched_vertices.find(*it) != previously_matched_vertices.end())
			continue;

		try
		{			
			Transform guess = mPoseGraph[*it].corrected_pose.inverse() * mPoseGraph[vertex].corrected_pose;
			TransformWithCovariance twc = sensor->calculateTransform(mPoseGraph[*it].measurement, mPoseGraph[vertex].measurement, guess);
			addEdge(*it, vertex, twc.transform, twc.covariance, sensor->getName(), "match");
			added++;
		}catch(NoMatch &e)
		{
			continue;
		}
	}
}

VertexObjectList BoostMapper::getVertexObjectsFromSensor(const std::string& sensor)
{
	VertexObjectList objectList;
	VertexRange vertices = boost::vertices(mPoseGraph);
	for(VertexIterator it = vertices.first; it != vertices.second; ++it)
	{
		if(mPoseGraph[*it].measurement->getSensorName() == sensor)
		{
			objectList.push_back(mPoseGraph[*it]);
		}
	}
	return objectList;
}

const VertexObject& BoostMapper::getVertex(IdType id)
{
	return mPoseGraph[mIndexMap[id]];
}
