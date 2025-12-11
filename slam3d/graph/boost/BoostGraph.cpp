#include "BoostGraph.hpp"

#include <slam3d/core/Solver.hpp>

#include <boost/format.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graphviz.hpp>

#include <fstream>

using namespace slam3d;

BoostGraph::BoostGraph(Logger* log, MeasurementStorage* storage)
 : Graph(log, storage)
{
	// insert a dummy node as a source of unary edges
	mIndexMap.insert(IndexMap::value_type(0, 0));
}

BoostGraph::~BoostGraph()
{
}

const EdgeObjectList BoostGraph::getEdgesFromSensor(const std::string& sensor) const
{
	EdgeObjectList objectList;
	EdgeRange edges = boost::edges(mPoseGraph);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		EdgeObject eo = mPoseGraph[*it];
		IdType source_id = mPoseGraph[boost::source(*it, mPoseGraph)].index;
		bool add_sensor = (sensor == "") || (sensor == eo.constraint->getSensorName());
		if(add_sensor && eo.source == source_id)
		{
			objectList.push_back(mPoseGraph[*it]);
		}
	}
	return objectList;
}

bool BoostGraph::optimize(unsigned iterations)
{
	boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
	return Graph::optimize(iterations);
}

void BoostGraph::addVertex(const VertexObject& v)
{
	boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);

	// Add vertex to the graph
	Vertex newVertex = boost::add_vertex(mPoseGraph);
	mPoseGraph[newVertex] = v;

	// Add it to the vertex index, so we can find it by its descriptor
	mIndexMap.insert(IndexMap::value_type(v.index, newVertex));
}

void BoostGraph::setVertex(IdType id, const VertexObject& v)
{
	mPoseGraph[mIndexMap.at(id)] = v;
}

void BoostGraph::addEdge(const EdgeObject& e)
{
	boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
	Edge forward_edge, inverse_edge;
	bool inserted_forward, inserted_inverse;
	
	Vertex source = mIndexMap.at(e.source);
	Vertex target = mIndexMap.at(e.target);
	boost::tie(forward_edge, inserted_forward) = boost::add_edge(source, target, mPoseGraph);
	boost::tie(inverse_edge, inserted_inverse) = boost::add_edge(target, source, mPoseGraph);

	if(inserted_forward && inserted_inverse)
	{
		mPoseGraph[forward_edge] = e;
		mPoseGraph[inverse_edge] = e;
	}else
	{
		mLogger->message(WARNING, (boost::format("Could not add an edge (%1%,%2%) to the BoostGraph.") % e.source % e.target).str());
		throw InvalidEdge(e.source, e.target);
	}
}

void BoostGraph::removeEdge(IdType source, IdType target, const std::string& sensor)
{
	boost::remove_edge(getEdgeIterator(source, target, sensor), mPoseGraph);
}

const VertexObjectList BoostGraph::getVerticesFromSensor(const std::string& sensor) const
{
	VertexObjectList objectList;
	VertexRange vertices = boost::vertices(mPoseGraph);
	for(VertexIterator it = vertices.first; it != vertices.second; ++it)
	{
		if(mPoseGraph[*it].sensorName == sensor)
		{
			objectList.push_back(mPoseGraph[*it]);
		}
	}
	return objectList;
}

const VertexObjectList BoostGraph::getVerticesByType(const std::string& type) const
{
	VertexObjectList objectList;
	VertexRange vertices = boost::vertices(mPoseGraph);
	for(VertexIterator it = vertices.first; it != vertices.second; ++it)
	{
		if(mPoseGraph[*it].typeName == type)
		{
			objectList.push_back(mPoseGraph[*it]);
		}
	}
	return objectList;
}

const VertexObject BoostGraph::getVertex(IdType id) const
{
	return mPoseGraph[mIndexMap.at(id)];
}

const EdgeObject BoostGraph::getEdge(IdType source, IdType target, const std::string& sensor) const
{
	OutEdgeIterator it = getEdgeIterator(source, target, sensor);
	return mPoseGraph[*it];
}

OutEdgeIterator BoostGraph::getEdgeIterator(IdType source, IdType target, const std::string& sensor) const
{
	OutEdgeIterator it, it_end;
	try
	{
		boost::tie(it, it_end) = boost::out_edges(mIndexMap.at(source), mPoseGraph);
	}catch(std::out_of_range &e)
	{
		throw InvalidVertex(source);
	}
	while(it != it_end)
	{
		const VertexObject& tObject = mPoseGraph[boost::target(*it, mPoseGraph)];
		if(tObject.index == target && mPoseGraph[*it].constraint->getSensorName() == sensor)
		{
			return it;
		}
		++it;
	}
	throw InvalidEdge(source, target);
}

const EdgeObjectList BoostGraph::getOutEdges(IdType source) const
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

const EdgeObjectList BoostGraph::getEdges(const VertexObjectList& vertices) const
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
	boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
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
	EdgeFilter(const AdjacencyGraph* g) : graph(g) {}
	bool operator()(const Edge& e) const
	{
		return (*graph)[e].constraint->getType() == SE3;
	}
	
	const AdjacencyGraph* graph;
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

const VertexObjectList BoostGraph::getVerticesInRange(IdType source_id, unsigned range) const
{
	// Create required data structures
	Vertex source = mIndexMap.at(source_id);
	DepthMap depth_map;
	depth_map[source] = 0;
	ColorMap c_map;
	MaxDepthVisitor vis(depth_map, range);
	
	// Do BFS on filtered graph
	FilteredGraph fg(mPoseGraph, EdgeFilter(&mPoseGraph));
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

const VertexObjectList BoostGraph::getAllVertices() const
{
	VertexObjectList vertice_list;
	vertice_list.reserve(num_vertices(mPoseGraph));
	for (auto entry : boost::make_iterator_range(vertices(mPoseGraph))) {
		vertice_list.push_back(mPoseGraph[entry]);
	}
	return vertice_list;
}

float BoostGraph::calculateGraphDistance(IdType source_id, IdType target_id) const
{
	boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
	int num = boost::num_vertices(mPoseGraph);
	std::vector<Vertex> parent(num);
	std::vector<float> distance(num);
	std::map<Edge, float> weight;
	EdgeRange edges = boost::edges(mPoseGraph);
	EdgeFilter filter(&mPoseGraph);
	for(EdgeIterator it = edges.first; it != edges.second; ++it)
	{
		if(filter(*it))
			weight[*it] = 1.0;
		else
			weight[*it] = 10000;
	}
	
	boost::dijkstra_shortest_paths(mPoseGraph, mIndexMap.at(source_id),
		boost::distance_map(boost::make_iterator_property_map(distance.begin(), boost::get(boost::vertex_index, mPoseGraph)))
		.predecessor_map(boost::make_iterator_property_map(parent.begin(), boost::get(boost::vertex_index, mPoseGraph)))
		.weight_map(boost::make_assoc_property_map(weight)) );

	return distance[mIndexMap.at(target_id)];
}

void BoostGraph::setCorrectedPose(IdType id, const Transform& pose)
{
	mPoseGraph[mIndexMap.at(id)].correctedPose = pose;
}
