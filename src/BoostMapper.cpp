#include "BoostMapper.hpp"
#include "Solver.hpp"

#include <boost/format.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graphviz.hpp>

#include <fstream>

using namespace slam3d;

BoostMapper::BoostMapper(Logger* log)
 : GraphMapper(log), mNeighborIndex(flann::KDTreeSingleIndexParams())
{
	// Add root node to the graph
	Measurement::Ptr origin(new MapOrigin());
	IdType id = mIndexer.getNext();
	Vertex root = boost::add_vertex(mPoseGraph);
	mPoseGraph[root].index = id;
	mPoseGraph[root].label = "root";
	mPoseGraph[root].corrected_pose = Transform::Identity();
	mPoseGraph[root].measurement = origin;

	// Add it to the indexes, so we can find it by its id and uuid
	mIndexMap.insert(IndexMap::value_type(id, root));
	mVertexIndex.insert(UuidMap::value_type(origin->getUniqueId(), root));

	mLastVertex = 0;
}

BoostMapper::~BoostMapper()
{
	
}

Transform BoostMapper::getCurrentPose()
{
	if(mLastVertex)
	{
		return mPoseGraph[mLastVertex].corrected_pose * mCurrentPose;
	}
	return mCurrentPose;
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

EdgeObjectList BoostMapper::getEdgeObjectsFromSensor(const std::string& sensor) const
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
	mLogger->message(DEBUG, (boost::format("Doing NN search from (%1%, %2%, %3%) with radius %4%.")%t[0]%t[1]%t[2]%radius).str());
	
	// Find points nearby
	std::vector< std::vector<int> > neighbors;
	std::vector< std::vector<NeighborIndex::DistanceType> > distances;
	int found = mNeighborIndex.radiusSearch(query, neighbors, distances, radius*radius, mSearchParams);
	
	// Write the result
	VertexList result;
	std::vector<int>::iterator it = neighbors[0].begin();
	std::vector<NeighborIndex::DistanceType>::iterator d = distances[0].begin();
	for(; it < neighbors[0].end(); ++it, ++d)
	{
		Vertex n = mNeighborMap[*it];
		result.push_back(n);
		mLogger->message(DEBUG, (boost::format(" - vertex %1% nearby (d = %2%)") % mPoseGraph[n].index % *d).str());
	}
	
	mLogger->message(DEBUG, (boost::format("Neighbor search found %1% vertices nearby.") % found).str());
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
		try
		{
			Vertex v = mIndexMap.at(id);
			mPoseGraph[v].corrected_pose = tf;
		}catch(std::out_of_range &e)
		{
			mLogger->message(ERROR, (boost::format("Vertex with id %1% does not exist!") % id).str());
		}
	}
	return true;
}

bool BoostMapper::addReading(Measurement::Ptr m)
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
		// Add real vertex and link it to root
		Vertex root = mIndexMap.at(0);
		Transform newPose = Transform::Identity();
		newPose.linear() = odometry.linear();
		mLastVertex = addVertex(m, newPose);
		mLastOdometricPose = odometry;
		mLogger->message(INFO, "Added first node to the graph.");
		addEdge(root, mLastVertex, newPose, Covariance::Identity() * 1000, "none", "root-link");
		buildNeighborIndex(sensor->getName());
		linkToNeighbors(mLastVertex, sensor, mMaxNeighorLinks);
		return true;
	}

	// Now we have a node, that is not the first and has not been added yet
	Vertex newVertex = 0;
	Transform odom_dist;
	
	if(mOdometry)
	{
		odom_dist = orthogonalize(mLastOdometricPose.inverse() * odometry);
		mCurrentPose = odom_dist;
		if(!checkMinDistance(odom_dist))
			return false;
	}
	
	if(mAddOdometryEdges)
	{
		// Add the vertex to the pose graph
		newVertex = addVertex(m, orthogonalize(mPoseGraph[mLastVertex].corrected_pose * mCurrentPose));

		// Add an edge representing the odometry information
		Covariance odom_cov = mOdometry->calculateCovariance(odom_dist);
		addEdge(mLastVertex, newVertex, odom_dist, odom_cov, "Odometry", "odom");
	}
	
	// Add edge to previous measurement
	try
	{
		TransformWithCovariance twc = sensor->calculateTransform(mPoseGraph[mLastVertex].measurement, m, mCurrentPose);
		mCurrentPose = twc.transform;
		
		if(newVertex)
		{
			mPoseGraph[newVertex].corrected_pose = orthogonalize(mPoseGraph[mLastVertex].corrected_pose * twc.transform);
		}else
		{
			if(!checkMinDistance(twc.transform))
				return false;
			newVertex = addVertex(m, orthogonalize(mPoseGraph[mLastVertex].corrected_pose * twc.transform));
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

	// Add edges to other measurements nearby
	buildNeighborIndex(sensor->getName());
	linkToNeighbors(newVertex, sensor, mMaxNeighorLinks);

	// Overall last vertex
	mLastVertex = newVertex;
	mLastOdometricPose = odometry;
	mCurrentPose = Transform::Identity();
	return true;
}

Vertex BoostMapper::addVertex(Measurement::Ptr m, const Transform &corrected)
{
	// Create the new VertexObject and add it to the PoseGraph
	IdType id = mIndexer.getNext();
	boost::format v_name("%1%:%2%(%3%)");
	v_name % m->getRobotName() % m->getSensorName() % id;
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
	
	mLogger->message(INFO, (boost::format("Created vertex %1% (from %2%:%3%).") % id % m->getRobotName() % m->getSensorName()).str());
	return newVertex;
}

void BoostMapper::addEdge(Vertex source, Vertex target,
	const Transform &t, const Covariance &c, const std::string& sensor, const std::string& label)
{
	Edge forward_edge, inverse_edge;
	bool inserted_forward, inserted_inverse;
	boost::tie(forward_edge, inserted_forward) = boost::add_edge(source, target, mPoseGraph);
	boost::tie(inverse_edge, inserted_inverse) = boost::add_edge(target, source, mPoseGraph);
	
	unsigned source_id = mPoseGraph[source].index;
	unsigned target_id = mPoseGraph[target].index;

	mPoseGraph[forward_edge].transform = t;
	mPoseGraph[forward_edge].covariance = c;
	mPoseGraph[forward_edge].sensor = sensor;
	mPoseGraph[forward_edge].label = label;
	mPoseGraph[forward_edge].source = source_id;
	mPoseGraph[forward_edge].target = target_id;
	
	mPoseGraph[inverse_edge].transform = t.inverse();
	mPoseGraph[inverse_edge].covariance = c;
	mPoseGraph[inverse_edge].sensor = sensor;
	mPoseGraph[inverse_edge].label = label;
	mPoseGraph[inverse_edge].source = target_id;
	mPoseGraph[inverse_edge].target = source_id;
	
	if(mSolver)
	{
		mSolver->addConstraint(source_id, target_id, t, c);
	}
	mLogger->message(INFO, (boost::format("Created '%4%' edge from node %1% to node %2% (from %3%).") % source_id % target_id % sensor % label).str());
}

TransformWithCovariance BoostMapper::link(Vertex source, Vertex target, Sensor* sensor)
{
	if(mPoseGraph[target].measurement->getSensorName() != sensor->getName())
	{
		throw InvalidEdge(mPoseGraph[source].index, mPoseGraph[target].index);
	}
	
	// Create virtual measurement for source node
	Transform sourcePose = mPoseGraph[source].corrected_pose;
	Transform targetPose = mPoseGraph[target].corrected_pose;
	
	Measurement::Ptr source_m = mPoseGraph[source].measurement;
	Measurement::Ptr target_m = mPoseGraph[target].measurement;
	
	if(mPatchBuildingRange > 0)
	{
		VertexList sourceVertices = getVerticesInRange(source, 3);
		VertexObjectList sourceObjects;
		for(VertexList::iterator it = sourceVertices.begin(); it != sourceVertices.end(); ++it)
		{
			if(mPoseGraph[*it].measurement->getSensorName() == sensor->getName())
				sourceObjects.push_back(mPoseGraph[*it]);
		}
		source_m = sensor->createCombinedMeasurement(sourceObjects, sourcePose);

		// Create virtual measurement for target node
		VertexList targetVertices = getVerticesInRange(target, 3);
		VertexObjectList targetObjects;
		for(VertexList::iterator it = targetVertices.begin(); it != targetVertices.end(); ++it)
		{
			if(mPoseGraph[*it].measurement->getSensorName() == sensor->getName())
				targetObjects.push_back(mPoseGraph[*it]);
		}
		target_m = sensor->createCombinedMeasurement(targetObjects, targetPose);
	}
	
	// Estimate the transform from source to target
	Transform guess = sourcePose.inverse() * targetPose;
	TransformWithCovariance twc = sensor->calculateTransform(source_m, target_m, guess);

	// Create new edge and return the transform
	addEdge(source, target, twc.transform, twc.covariance, sensor->getName(), "loop");
	return twc;
}

void BoostMapper::addExternalReading(Measurement::Ptr m, boost::uuids::uuid s, const Transform& tf, const Covariance& cov, const std::string& sensor)
{
	if(mVertexIndex.find(m->getUniqueId()) != mVertexIndex.end())
	{
		throw DuplicateMeasurement();
	}
	
	Vertex source = mVertexIndex.at(s);
	Transform pose = mPoseGraph[source].corrected_pose * tf;
	Vertex target = addVertex(m, pose);
	addEdge(source, target, tf, cov, sensor, "ext");
}

void BoostMapper::addExternalConstraint(boost::uuids::uuid s, boost::uuids::uuid t, const Transform& tf, const Covariance& cov, const std::string& sensor)
{
	Vertex source = mVertexIndex.at(s);
	Vertex target = mVertexIndex.at(t);
	try
	{
		IdType s_id = mPoseGraph[source].index;
		IdType t_id = mPoseGraph[target].index;
		getEdge(s_id, t_id, sensor);
		throw DuplicateEdge(s_id, t_id, sensor);
	}catch(InvalidEdge &ie)
	{
		addEdge(source, target, tf, cov, sensor, "ext");
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
	
	std::vector<Vertex> neighbors = getNearbyVertices(mPoseGraph[vertex].corrected_pose, mNeighborRadius);
	
	int added = 0;
	for(std::vector<Vertex>::iterator it = neighbors.begin(); it != neighbors.end() && added < max_links; ++it)
	{
		if(previously_matched_vertices.find(*it) != previously_matched_vertices.end())
			continue;

		try
		{
			float dist = calculateGraphDistance(*it, vertex);
			mLogger->message(DEBUG, (boost::format("Distance(%2%,%3%) in Graph is: %1%") % dist % mPoseGraph[*it].index % mPoseGraph[vertex].index).str());
			if(dist < mPatchBuildingRange * 2)
				continue;
			link(*it, vertex, sensor);
			added++;
		}catch(NoMatch &e)
		{
			continue;
		}
	}
}

VertexObjectList BoostMapper::getVertexObjectsFromSensor(const std::string& sensor) const
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

const VertexObject& BoostMapper::getVertex(IdType id) const
{
	return mPoseGraph[mIndexMap.at(id)];
}

const VertexObject& BoostMapper::getVertex(boost::uuids::uuid id) const
{
	return mPoseGraph[mVertexIndex.at(id)];
}

const EdgeObject& BoostMapper::getEdge(IdType source, IdType target, const std::string& sensor) const
{
	OutEdgeIterator it, it_end;
	boost::tie(it, it_end) = boost::out_edges(mIndexMap.at(source), mPoseGraph);
	while(it != it_end)
	{
		const VertexObject& tObject = mPoseGraph[boost::target(*it, mPoseGraph)];
		if(tObject.index == target && mPoseGraph[*it].sensor == sensor)
		{
			return mPoseGraph[*it];
		}
		++it;
	}
	throw InvalidEdge(source, target);
}

EdgeObjectList BoostMapper::getOutEdges(IdType source) const
{
	OutEdgeIterator it, it_end;
	boost::tie(it, it_end) = boost::out_edges(mIndexMap.at(source), mPoseGraph);
	EdgeObjectList edges;
	while(it != it_end)
	{
		edges.push_back(mPoseGraph[*it]);
		++it;
	}
	return edges;
}

void BoostMapper::writeGraphToFile(const std::string& name)
{
	std::string file = name + ".dot";
	mLogger->message(INFO, (boost::format("Writing graph to file '%1%'.") % file).str());
	std::ofstream ofs;
	ofs.open(file.c_str());
	boost::write_graphviz(
			ofs,
			mPoseGraph,
			boost::make_label_writer(boost::get(&VertexObject::label, mPoseGraph)),
			boost::make_label_writer(boost::get(&EdgeObject::label, mPoseGraph)),
			boost::default_writer(),
			boost::get(&VertexObject::index, mPoseGraph));
	ofs.close();
}

// ================================================================
// BFS search for vertices with a maximum distance to a source node
// ================================================================

struct EdgeFilter
{
	EdgeFilter() {}
	EdgeFilter(AdjacencyGraph* g, std::string n) : graph(g), name(n) {}
	bool operator()(const Edge& e) const
	{
		return (*graph)[e].label == name;
	}
	
	AdjacencyGraph* graph;
	std::string name;
};

typedef boost::filtered_graph<AdjacencyGraph, EdgeFilter> FilteredGraph;
typedef std::map<FilteredGraph::vertex_descriptor, boost::default_color_type> ColorMap;
typedef std::map<FilteredGraph::vertex_descriptor, unsigned> DepthMap;

/**
 * @class MaxDepthVisitor
 * @brief BFS-Visitor to find nearby nodes in the graph.
 */
class MaxDepthVisitor : public boost::default_bfs_visitor
{
public:
	MaxDepthVisitor(DepthMap& map, unsigned d) : depth_map(map), max_depth(d) {}

	void tree_edge(FilteredGraph::edge_descriptor e, const FilteredGraph& g)
	{
		FilteredGraph::vertex_descriptor u = source(e, g);
		FilteredGraph::vertex_descriptor v = target(e, g);
		if(depth_map[u] >= max_depth)
			throw 0;
		depth_map[v] = depth_map[u] + 1;
	}
private:
	DepthMap& depth_map;
	unsigned max_depth;
};

VertexList BoostMapper::getVerticesInRange(Vertex source, unsigned range)
{
	// Create required data structures
	DepthMap depth_map;
	depth_map[source] = 0;
	ColorMap c_map;
	MaxDepthVisitor vis(depth_map, range);
	
	// Do BFS on filtered graph
	FilteredGraph fg(mPoseGraph, EdgeFilter(&mPoseGraph, "seq"));
	try
	{
		boost::breadth_first_search(fg, source, boost::visitor(vis).color_map(boost::associative_property_map<ColorMap>(c_map)));
	}catch(int e)
	{
	}

	// Write the result
	VertexList vertices;
	for(DepthMap::iterator it = depth_map.begin(); it != depth_map.end(); ++it)
	{
		vertices.push_back(it->first);
	}
	return vertices;
}

float BoostMapper::calculateGraphDistance(Vertex source, Vertex target)
{
	int num = boost::num_vertices(mPoseGraph);
	std::vector<Vertex> parent(num);
	std::vector<float> distance(num);
	std::map<Edge, float> weight;
	EdgeRange edges = boost::edges(mPoseGraph);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		weight[*it] = 1.0;
	}
	
	boost::dijkstra_shortest_paths(mPoseGraph, source,
		boost::distance_map(boost::make_iterator_property_map(distance.begin(), boost::get(boost::vertex_index, mPoseGraph)))
		.predecessor_map(boost::make_iterator_property_map(parent.begin(), boost::get(boost::vertex_index, mPoseGraph)))
		.weight_map(boost::make_assoc_property_map(weight)) );

	int target_id = boost::get(boost::vertex_index, mPoseGraph)[target];
	return distance[target_id];
}