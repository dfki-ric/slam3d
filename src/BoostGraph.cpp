// workaround for:
//https://svn.boost.org/trac/boost/ticket/10382
#define BOOST_NO_CXX11_DEFAULTED_FUNCTIONS


#include "BoostGraph.hpp"
#include "Solver.hpp"

#include <boost/format.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graphviz.hpp>

#include <fstream>

using namespace slam3d;

BoostGraph::BoostGraph(Logger* log)
 : Graph(log)
{
}

BoostGraph::~BoostGraph()
{
}

const VertexObject& BoostGraph::getLastVertex() const
{
	return mPoseGraph[mIndexMap.at(mLastIndex)];
}

VertexList BoostGraph::getVerticesFromSensor(const std::string& sensor)
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

EdgeList BoostGraph::getEdgesFromSensor(const std::string& sensor)
{
	EdgeList edgeList;
	EdgeRange edges = boost::edges(mPoseGraph);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		edgeList.push_back(*it);
	}
	return edgeList;
}

EdgeObjectList BoostGraph::getEdgesFromSensor(const std::string& sensor) const
{
	EdgeObjectList objectList;
	EdgeRange edges = boost::edges(mPoseGraph);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		objectList.push_back(mPoseGraph[*it]);
	}
	return objectList;
}

bool BoostGraph::optimize()
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
			Vertex v = mIndexMap.at(id);
			mPoseGraph[v].corrected_pose = tf;
		}catch(std::out_of_range &e)
		{
			mLogger->message(ERROR, (boost::format("Vertex with id %1% does not exist!") % id).str());
		}
	}
	return true;
}

IdType BoostGraph::addVertex(Measurement::Ptr m, const Transform &corrected)
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
	mUuidIndex.insert(UuidIndex::value_type(m->getUniqueId(), id));
	
	// Add it to the SLAM-Backend for incremental optimization
	if(mSolver)
	{
		mSolver->addNode(id, corrected);
	}
	
	mLogger->message(INFO, (boost::format("Created vertex %1% (from %2%:%3%).") % id % m->getRobotName() % m->getSensorName()).str());
	return id;
}

void BoostGraph::addConstraint(IdType source_id, IdType target_id,
	const Transform &t, const Covariance &c, const std::string& sensor, const std::string& label)
{
	Edge forward_edge, inverse_edge;
	bool inserted_forward, inserted_inverse;
	
	Vertex source = mIndexMap[source_id];
	Vertex target = mIndexMap[target_id];
	boost::tie(forward_edge, inserted_forward) = boost::add_edge(source, target, mPoseGraph);
	boost::tie(inverse_edge, inserted_inverse) = boost::add_edge(target, source, mPoseGraph);

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

VertexObjectList BoostGraph::getVerticesFromSensor(const std::string& sensor) const
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

const VertexObject& BoostGraph::getVertex(IdType id) const
{
	return mPoseGraph[mIndexMap.at(id)];
}

const EdgeObject& BoostGraph::getEdge(IdType source, IdType target, const std::string& sensor) const
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

EdgeObjectList BoostGraph::getOutEdges(IdType source) const
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

EdgeObjectList BoostGraph::getEdges(const VertexObjectList& vertices)
{
	std::set<int> v_ids;
	for(VertexObjectList::const_iterator v = vertices.begin(); v != vertices.end(); v++)
	{
		v_ids.insert(v->index);
	}
	EdgeObjectList objectList;
	EdgeRange edges = boost::edges(mPoseGraph);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		EdgeObject ed = mPoseGraph[*it];
		if(v_ids.find(ed.source) != v_ids.end() && v_ids.find(ed.target) != v_ids.end())
			objectList.push_back(ed);
	}
	return objectList;
}

void BoostGraph::writeGraphToFile(const std::string& name)
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
	EdgeFilter(const AdjacencyGraph* g, std::string n) : graph(g), name(n) {}
	bool operator()(const Edge& e) const
	{
		return (*graph)[e].sensor == name;
	}
	
	const AdjacencyGraph* graph;
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

VertexObjectList BoostGraph::getVerticesInRange(IdType source_id, unsigned range) const
{
	// Create required data structures
	Vertex source = mIndexMap.at(source_id);
	DepthMap depth_map;
	depth_map[source] = 0;
	ColorMap c_map;
	MaxDepthVisitor vis(depth_map, range);
	
	// Do BFS on filtered graph
	std::string sensor = mPoseGraph[source].measurement->getSensorName();
	FilteredGraph fg(mPoseGraph, EdgeFilter(&mPoseGraph, sensor));
	try
	{
		boost::breadth_first_search(fg, source, boost::visitor(vis).color_map(boost::associative_property_map<ColorMap>(c_map)));
	}catch(int e)
	{
	}

	// Write the result
	VertexObjectList vertices;
	for(DepthMap::iterator it = depth_map.begin(); it != depth_map.end(); ++it)
	{
		vertices.push_back(mPoseGraph[it->first]);
	}
	return vertices;
}

float BoostGraph::calculateGraphDistance(IdType source_id, IdType target_id)
{
	int num = boost::num_vertices(mPoseGraph);
	std::vector<Vertex> parent(num);
	std::vector<float> distance(num);
	std::map<Edge, float> weight;
	EdgeRange edges = boost::edges(mPoseGraph);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		if(mPoseGraph[*it].sensor == "none")
			weight[*it] = 100.0;
		else
			weight[*it] = 1.0;
	}
	
	boost::dijkstra_shortest_paths(mPoseGraph, mIndexMap.at(source_id),
		boost::distance_map(boost::make_iterator_property_map(distance.begin(), boost::get(boost::vertex_index, mPoseGraph)))
		.predecessor_map(boost::make_iterator_property_map(parent.begin(), boost::get(boost::vertex_index, mPoseGraph)))
		.weight_map(boost::make_assoc_property_map(weight)) );

	return distance[target_id];
}
/*
VertexObjectList BoostGraph::getNearbyUnlinkedVertices(const Transform &tf, float radius, const std::string &sensor) const
{
	VertexObjectList objects;
	VertexList neighbors = getNearbyVertices(tf, radius);
	for(VertexList::iterator it = neighbors.begin(); it < neighbors.end(); it++)
	{
//		const VertexObject obj = mPoseGraph[*it];
		objects.push_back(mPoseGraph[*it]);
	}
	return objects;
}
*/