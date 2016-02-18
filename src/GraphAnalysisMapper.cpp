#include "GraphAnalysisMapper.hpp"
#include "Solver.hpp"

#include <boost/format.hpp>
#include <graph_analysis/lemon/DirectedGraph.hpp>
#include <graph_analysis/GraphIO.hpp>

#define PI 3.141592654

using namespace graph_analysis;

/*****************************************************************************
 * Interface methods
 *****************************************************************************/
 
 Mapper::Mapper(slam3d::Logger* log) : slam3d::GraphMapper(log),
   mPoseGraph( new lemon::DirectedGraph()),
   mIndex(flann::KDTreeSingleIndexParams())
{
	mOdometry = NULL;
	mSolver = NULL;
	mLogger = log;
	
	mNeighborRadius = 1.0;
	mMinTranslation = 0.5;
	mMinRotation = 0.1;
	mAddOdometryEdges = false;

	mCurrentPose = slam3d::Transform::Identity();
}

Mapper::~Mapper()
{
}

bool Mapper::addReading(slam3d::Measurement* m)
{
	// Get the sensor responsible for this measurement
	// Can throw std::out_of_range if sensor is not registered
	slam3d::Sensor* sensor = NULL;
	try
	{
		sensor = mSensors.at(m->getSensorName());
		mLogger->message(slam3d::DEBUG, (boost::format("Add reading from own Sensor '%1%'.") % m->getSensorName()).str());
	}catch(std::out_of_range e)
	{
		mLogger->message(slam3d::ERROR, (boost::format("Sensor '%1%' has not been registered!") % m->getSensorName()).str());
		return false;
	}

	// Get the odometric pose for this measurement
	slam3d::Transform odometry = slam3d::Transform::Identity();
	if(mOdometry)
	{
		try
		{
			odometry = mOdometry->getOdometricPose(m->getTimestamp());
		}catch(slam3d::OdometryException &e)
		{
			mLogger->message(slam3d::ERROR, "Could not get Odometry data!");
			return false;
		}
	}

	// If this is the first vertex, add it and return
	if(!mLastVertex)
	{
		mLastVertex = addVertex(m, mCurrentPose);
		mLastOdometricPose = odometry;
		mLogger->message(slam3d::INFO, "Added first node to the graph.");
		return true;
	}

	// Now we have a node, that is not the first and has not been added yet
	VertexObject::Ptr newVertex;
	
	if(mOdometry)
	{
		slam3d::Transform odom_dist = orthogonalize(mLastOdometricPose.inverse() * odometry);
		mCurrentPose = mLastVertex->corrected_pose * odom_dist;
		if(!checkMinDistance(odom_dist))
			return false;

		if(mAddOdometryEdges)
		{
			// Add the vertex to the pose graph
			newVertex = addVertex(m, orthogonalize(mCurrentPose));

			// Add an edge representing the odometry information
			addEdge(mLastVertex, newVertex, odom_dist, slam3d::Covariance::Identity(), "Odometry", "odom");
		}
	}

	// Add edge to previous measurement
	if(mLastVertex)
	{
		try
		{
			slam3d::Transform guess = mLastVertex->corrected_pose.inverse() * mCurrentPose;
			slam3d::TransformWithCovariance twc = sensor->calculateTransform(mLastVertex->measurement, m, guess);
			mCurrentPose = orthogonalize(mLastVertex->corrected_pose * twc.transform);
			
			if(newVertex)
			{
				newVertex->corrected_pose = mCurrentPose;
			}else
			{
				if(!checkMinDistance(twc.transform))
					return false;
				newVertex = addVertex(m, mCurrentPose);
			}
			addEdge(mLastVertex, newVertex, twc.transform, twc.covariance, sensor->getName(), "seq");
		}catch(slam3d::NoMatch &e)
		{
			if(!newVertex)
			{
				mLogger->message(slam3d::WARNING, "Measurement could not be matched and no odometry was availabe!");
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


void Mapper::addExternalReading(slam3d::Measurement* m, const slam3d::Transform& t)
{
	VertexObject::Ptr v = addVertex(m, t);
		
	// Get the sensor responsible for this measurement
	// Can throw std::out_of_range if sensor is not registered
	mLogger->message(slam3d::DEBUG, (boost::format("Add external reading from %1%:%2%.") % m->getRobotName() % m->getSensorName()).str());
	slam3d::Sensor* sensor = NULL;
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

bool Mapper::optimize()
{
	if(!mSolver)
	{
		mLogger->message(slam3d::ERROR, "A solver must be set before optimize() is called!");
		return false;
	}

	// Optimize
	if(!mSolver->compute())
	{
		return false;
	}

	// Retrieve results
	slam3d::IdPoseVector res = mSolver->getCorrections();
	for(slam3d::IdPoseVector::iterator it = res.begin(); it < res.end(); it++)
	{
		unsigned int id = it->first;
		slam3d::Transform tf = it->second;
		VertexObject::Ptr v = fromBaseGraph(mPoseGraph->getVertex(id));
		v->corrected_pose = tf;
	}
	
	if(mLastVertex)
	{
		mCurrentPose = mLastVertex->corrected_pose;
	}
	return true;
}

const slam3d::VertexObject& Mapper::getLastVertex()
{
	return (*mLastVertex);
}

const slam3d::VertexObject& Mapper::getVertex(slam3d::IdType id)
{
	return *fromBaseGraph(mPoseGraph->getVertex(id));
}

slam3d::VertexObjectList Mapper::getVertexObjectsFromSensor(const std::string& sensor)
{
	VertexList vertices = getVerticesFromSensor(sensor);
	slam3d::VertexObjectList objects;
	for(VertexList::iterator it = vertices.begin(); it != vertices.end(); ++it)
	{
		objects.push_back(**it);
	}
	return objects;
}

slam3d::EdgeObjectList Mapper::getEdgeObjectsFromSensor(const std::string& sensor)
{
	EdgeList edges = getEdgesFromSensor(sensor);
	slam3d::EdgeObjectList objects;
	for(EdgeList::iterator it = edges.begin(); it != edges.end(); ++it)
	{
		objects.push_back(**it);
	}
	return objects;
}

void Mapper::writeGraphToFile(const std::string &name)
{
	std::string file = name + ".dot";
	mLogger->message(slam3d::INFO, (boost::format("Writing graph to file '%1%'.") % file).str());
	io::GraphIO::write(file, *mPoseGraph, representation::GRAPHVIZ);
}

 /*****************************************************************************
 * Internal methods
 *****************************************************************************/

VertexObject::Ptr Mapper::fromBaseGraph(Vertex::Ptr base)
{
	VertexObject::Ptr v = boost::dynamic_pointer_cast<VertexObject>(base);
	if(!v)
	{
		throw BadElementType();
	}
	return v;
}

EdgeObject::Ptr Mapper::fromBaseGraph(Edge::Ptr base)
{
	EdgeObject::Ptr e = boost::dynamic_pointer_cast<EdgeObject>(base);
	if(!e)
	{
		throw BadElementType();
	}
	return e;
}

VertexObject::Ptr Mapper::addVertex(slam3d::Measurement* m, const slam3d::Transform &corrected)
{
	// Create the new VertexObject and add it to the PoseGraph
	boost::format v_name("%1%:%2%");
	v_name % m->getRobotName() % m->getSensorName();
	VertexObject::Ptr newVertex(new VertexObject(v_name.str()));
	newVertex->corrected_pose = corrected;
	newVertex->measurement = m;
	mPoseGraph->addVertex(newVertex);
	
	// Add it to the Index, so we can find it by its unique id
	mVertexIndex.insert(VertexIndex::value_type(m->getUniqueId(), newVertex));
	
	// Add it to the SLAM-Backend for incremental optimization
	if(mSolver)
	{
		GraphElementId id = mPoseGraph->getVertexId(newVertex);
		mLogger->message(slam3d::INFO, (boost::format("Created vertex %1% (from %2%:%3%).") % id % m->getRobotName() % m->getSensorName()).str());
		mSolver->addNode(id, newVertex->corrected_pose);
	}
	
	// Set it as fixed in the solver
	if(!mFirstVertex)
	{
		mFirstVertex = newVertex;
		if(mSolver)
		{
			GraphElementId id = mPoseGraph->getVertexId(newVertex);
			mSolver->setFixed(id);
		}
	}
	
	return newVertex;
}

EdgeObject::Ptr Mapper::addEdge(VertexObject::Ptr source, VertexObject::Ptr target,
	const slam3d::Transform &t, const slam3d::Covariance &c, const std::string& sensor, const std::string& label)
{
	EdgeObject::Ptr edge(new EdgeObject(sensor, label));
	edge->setSourceVertex(source);
	edge->setTargetVertex(target);
	edge->transform = t;
	edge->covariance = c;
	mPoseGraph->addEdge(edge);
	
	if(mSolver)
	{
		unsigned source_id = mPoseGraph->getVertexId(source);
		unsigned target_id = mPoseGraph->getVertexId(target);
		mLogger->message(slam3d::INFO, (boost::format("Created '%4%' edge from node %1% to node %2% (from %3%).") % source_id % target_id % sensor % label).str());
		mSolver->addConstraint(source_id, target_id, t, c);
	}
	return edge;
}

void Mapper::linkToNeighbors(VertexObject::Ptr vertex, slam3d::Sensor* sensor, int max_links)
{
	// Get all edges to/from this node
	EdgeIterator::Ptr edgeIterator = mPoseGraph->getEdgeIterator(vertex);
	std::set<Vertex::Ptr> previously_matched_vertices;
	previously_matched_vertices.insert(vertex);
	while(edgeIterator->next())
	{
		EdgeObject::Ptr edge = boost::dynamic_pointer_cast<EdgeObject>(edgeIterator->current());
		if(edge->sensor == sensor->getName())
		{
			if(edge->getSourceVertex() == vertex)
				previously_matched_vertices.insert(edge->getTargetVertex());
			else
				previously_matched_vertices.insert(edge->getSourceVertex());
		}
	}
	
	VertexList neighbors = getNearbyVertices(vertex->corrected_pose, mNeighborRadius);
	mLogger->message(slam3d::DEBUG, (boost::format("Neighbor search found %1% vertices nearby.") % neighbors.size()).str());
	
	int added = 0;
	for(VertexList::iterator it = neighbors.begin(); it < neighbors.end() && added < max_links; it++)
	{
		if(previously_matched_vertices.find(*it) != previously_matched_vertices.end())
			continue;

		try
		{			
			slam3d::Transform guess = (*it)->corrected_pose.inverse() * vertex->corrected_pose;
			slam3d::TransformWithCovariance twc = sensor->calculateTransform((*it)->measurement, vertex->measurement, guess);
			addEdge(*it, vertex, twc.transform, twc.covariance, sensor->getName(), "match");
			added++;
		}catch(slam3d::NoMatch &e)
		{
			continue;
		}
	}
}

VertexList Mapper::getVerticesFromSensor(const std::string& sensor)
{
	VertexList vertexList;

	VertexIterator::Ptr vertexIterator = mPoseGraph->getVertexIterator();
	while(vertexIterator->next())
	{
		VertexObject::Ptr vertex = boost::dynamic_pointer_cast<VertexObject>(vertexIterator->current());
		if(vertex->measurement->getSensorName() == sensor)
		{
			vertexList.push_back(vertex);
		}
	}
	return vertexList;
}

EdgeList Mapper::getEdgesFromSensor(const std::string& sensor)
{
	EdgeList edgeList;
	
	EdgeIterator::Ptr edgeIterator = mPoseGraph->getEdgeIterator();
	while(edgeIterator->next())
	{
		EdgeObject::Ptr edge = boost::dynamic_pointer_cast<EdgeObject>(edgeIterator->current());
		edgeList.push_back(edge);
	}
	
	return edgeList;
}

void Mapper::buildNeighborIndex(const std::string& sensor)
{
	VertexList vertices = getVerticesFromSensor(sensor);
	int numOfVertices = vertices.size();
	flann::Matrix<float> points(new float[numOfVertices * 3], numOfVertices, 3);

	int row = 0;
	mIndexMap.clear();
	for(VertexList::iterator it = vertices.begin(); it < vertices.end(); it++)
	{
		slam3d::Transform::TranslationPart t = (*it)->corrected_pose.translation();
		points[row][0] = t[0];
		points[row][1] = t[1];
		points[row][2] = t[2];
		mIndexMap.insert(std::pair<int, VertexObject::Ptr>(row, *it));
		row++;
	}
	
	mIndex.buildIndex(points);
}

VertexList Mapper::getNearbyVertices(const slam3d::Transform &tf, float radius)
{
	// Fill in the query point
	flann::Matrix<float> query(new float[3], 1, 3);
	slam3d::Transform::ConstTranslationPart t = tf.translation();
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
