#include "GraphMapper.hpp"
#include "Solver.hpp"

#include <boost/format.hpp>

#define PI 3.141592654

using namespace slam3d;

// Re-orthogonalize the rotation-matrix
// http://stackoverflow.com/questions/23080791/eigen-re-orthogonalization-of-rotation-matrix
Transform orthogonalize(const Transform& t)
{
	Vector3 x(t(0,0), t(0,1), t(0,2));
	Vector3 y(t(1,0), t(1,1), t(1,2));
	Vector3 z(t(2,0), t(2,1), t(2,2));
	ScalarType error = x.dot(y);
	
	Vector3 x_ort = x - (error/2.0) * y;
	Vector3 y_ort = y - (error/2.0) * x;
	Vector3 z_ort = x_ort.cross(y_ort);

	Transform res = t;
	ScalarType xdot = 0.5 * (3.0 - x_ort.dot(x_ort));
	res(0,0) = xdot * x_ort(0);
	res(0,1) = xdot * x_ort(1);
	res(0,2) = xdot * x_ort(2);
	
	ScalarType ydot = 0.5 * (3.0 - y_ort.dot(y_ort));
	res(1,0) = ydot * y_ort(0);
	res(1,1) = ydot * y_ort(1);
	res(1,2) = ydot * y_ort(2);
	
	ScalarType zdot = 0.5 * (3.0 - z_ort.dot(z_ort));
	res(2,0) = zdot * z_ort(0);
	res(2,1) = zdot * z_ort(1);
	res(2,2) = zdot * z_ort(2);
	
	return res;
}

GraphMapper::GraphMapper(Logger* log)
 : mIndex(flann::KDTreeSingleIndexParams())
{
	mOdometry = NULL;
	mSolver = NULL;
	mLogger = log;
	
	mNeighborRadius = 1.0;
	mMinTranslation = 0.5;
	mMinRotation = 0.1;
	mAddOdometryEdges = false;

	mCurrentPose = Transform::Identity();
	mLastVertex = 0;
	mFirstVertex = 0;
}

GraphMapper::~GraphMapper()
{
}

void GraphMapper::setSolver(Solver* solver)
{
	mSolver = solver;
}

void GraphMapper::setOdometry(Odometry* odom, bool add_edges)
{
	mOdometry = odom;
	mAddOdometryEdges = add_edges;
}

bool GraphMapper::optimize()
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
		Vertex v = mIndexMap[id];
		mPoseGraph[v].corrected_pose = tf;
	}
	
	if(mLastVertex)
	{
		mCurrentPose = mPoseGraph[mLastVertex].corrected_pose;
	}
	return true;
}

void GraphMapper::registerSensor(Sensor* s)
{
	std::pair<SensorList::iterator, bool> result;
	result = mSensors.insert(SensorList::value_type(s->getName(), s));
	if(!result.second)
	{
		mLogger->message(ERROR, (boost::format("Sensor with name %1% already exists!") % s->getName()).str());
		return;
	}
}

Vertex GraphMapper::addVertex(Measurement* m, const Transform &corrected)
{
	// Create the new VertexObject and add it to the PoseGraph
	boost::format v_name("%1%:%2%");
	v_name % m->getRobotName() % m->getSensorName();
	int id = mIndexer.getNext();
	Vertex newVertex = boost::add_vertex(mPoseGraph);
	mPoseGraph[newVertex].index = id;
	mPoseGraph[newVertex].label = v_name.str();
	mPoseGraph[newVertex].corrected_pose = corrected;
	mPoseGraph[newVertex].measurement = m;

	// Add it to the Index, so we can find it by its unique id
	mVertexIndex.insert(VertexIndex::value_type(m->getUniqueId(), newVertex));
	
	// Add it to the SLAM-Backend for incremental optimization
	if(mSolver)
	{
		mLogger->message(INFO, (boost::format("Created vertex %1% (from %2%:%3%).") % id % m->getRobotName() % m->getSensorName()).str());
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
	
	return newVertex;
}

Edge GraphMapper::addEdge(Vertex source, Vertex target,
	const Transform &t, const Covariance &c, const std::string& sensor, const std::string& label)
{
	Edge edge;
	bool inserted;
	boost::tie(edge, inserted) = boost::add_edge(source, target, mPoseGraph);

	mPoseGraph[edge].transform = t;
	mPoseGraph[edge].covariance = c;
	mPoseGraph[edge].sensor = sensor;
	mPoseGraph[edge].label = label;
	
	if(mSolver)
	{
		unsigned source_id = mPoseGraph[source].index;
		unsigned target_id = mPoseGraph[target].index;
		mLogger->message(INFO, (boost::format("Created '%4%' edge from node %1% to node %2% (from %3%).") % source_id % target_id % sensor % label).str());
		mSolver->addConstraint(source_id, target_id, t, c);
	}
	return edge;
}

bool GraphMapper::addReading(Measurement* m)
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

void GraphMapper::addExternalReading(Measurement* m, const Transform& t)
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
/*
std::vector<Edge> all_edges(Vertex v, AdjacencyGraph& g)
{
	std::vector<Edge> result;
	EdgeRange edges = boost::out_edges(v, g);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		result.push_back(*it);
	}
	
	edges = boost::in_edges(v, g);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		result.push_back(*it);
	}
	
	return result;
}
*/

void GraphMapper::linkToNeighbors(Vertex vertex, Sensor* sensor, int max_links)
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

VertexList GraphMapper::getVerticesFromSensor(const std::string& sensor)
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

VertexObjectList GraphMapper::getVertexObjectsFromSensor(const std::string& sensor)
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

EdgeList GraphMapper::getEdgesFromSensor(const std::string& sensor)
{
	EdgeList edgeList;
	EdgeRange edges = boost::edges(mPoseGraph);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		edgeList.push_back(*it);
	}
	
	return edgeList;
}

void GraphMapper::buildNeighborIndex(const std::string& sensor)
{
	VertexList vertices = getVerticesFromSensor(sensor);
	int numOfVertices = vertices.size();
	flann::Matrix<float> points(new float[numOfVertices * 3], numOfVertices, 3);

	int row = 0;
	mIndexMap.clear();
	for(VertexList::iterator it = vertices.begin(); it < vertices.end(); ++it)
	{
		Transform::TranslationPart t = mPoseGraph[*it].corrected_pose.translation();
		points[row][0] = t[0];
		points[row][1] = t[1];
		points[row][2] = t[2];
		mIndexMap.insert(std::pair<int, Vertex>(row, *it));
		row++;
	}
	
	mIndex.buildIndex(points);
}

VertexList GraphMapper::getNearbyVertices(const Transform &tf, float radius)
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
	mIndex.radiusSearch(query, neighbors, distances, radius, mSearchParams);
	
	// Write the result
	VertexList result;
	std::vector<int>::iterator it;
	for(it = neighbors[0].begin(); it < neighbors[0].end(); it++)
	{
		result.push_back(mIndexMap[*it]);
	}
	
	return result;
}

Transform GraphMapper::getCurrentPose()
{
	return mCurrentPose;
}

void GraphMapper::writeGraphToFile(const std::string &name)
{
	mLogger->message(ERROR, "Graph writing not implemented!");
}

bool GraphMapper::checkMinDistance(const Transform &t)
{
	ScalarType rot = Eigen::AngleAxis<ScalarType>(t.rotation()).angle();
	if(rot > PI)
		rot = (2*PI) - rot;
	if(rot < -PI)
		rot += 2*PI;
	ScalarType dx = t.translation()(0);
	ScalarType dy = t.translation()(1);
	ScalarType dz = t.translation()(2);
	ScalarType trans = sqrt(dx*dx + dy*dy + dz*dz);
	mLogger->message(DEBUG, (boost::format("Translation: %1% / Rotation: %2%") % trans % rot).str());
	if(trans < mMinTranslation && rot < mMinRotation)
		return false;
	else
		return true;
}
