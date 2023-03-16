// workaround for:
//https://svn.boost.org/trac/boost/ticket/10382
// #define BOOST_NO_CXX11_DEFAULTED_FUNCTIONS


#include "Neo4jGraph.hpp"

#include <boost/format.hpp>
#include <slam3d/core/Solver.hpp>

#include <fstream>

// #include <cpprest/uri.h>

#include <cpprest/http_client.h>
#include <cpprest/json.h>


using namespace slam3d;

using namespace web;
using namespace http;
using namespace utility;
using status_codes = web::http::status_codes;
// using Client = web::http::client::http_client;

Neo4jGraph::Neo4jGraph(Logger* log) : Graph(log)
{
    web::http::client::http_client_config clientconf;
    clientconf.set_validate_certificates(false);
    web::credentials clientcred(U("neo4j"), U("neo4j"));
    clientconf.set_credentials(clientcred);
    client = std::make_shared<web::http::client::http_client>(U("http://localhost:7474"), clientconf);

}

Neo4jGraph::~Neo4jGraph()
{
    // neo4j_close(connection);
    // neo4j_client_cleanup();
}

EdgeObjectList Neo4jGraph::getEdgesFromSensor(const std::string& sensor) const
{
	EdgeObjectList objectList;
	// EdgeRange edges = boost::edges(mPoseGraph);
	// for(EdgeIterator it = edges.first; it != edges.second; ++it)
	// {
	// 	EdgeObject eo = mPoseGraph[*it];
	// 	IdType source_id = mPoseGraph[boost::source(*it, mPoseGraph)].index;
	// 	bool add_sensor = (sensor == "") || (sensor == eo.constraint->getSensorName());
	// 	if(add_sensor && eo.source == source_id)
	// 	{
	// 		objectList.push_back(mPoseGraph[*it]);
	// 	}
	// }
	return objectList;
}

bool Neo4jGraph::optimize(unsigned iterations)
{
	boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
	return Graph::optimize(iterations);
}

void Neo4jGraph::addVertex(const VertexObject& v)
{
    std::string label = v.label;

    std::vector<std::string> props;
    props.push_back("\"props\": {");
    props.push_back("\"label\": \""+label+"\",");
    props.push_back("\"index\": \""+std::to_string(v.index)+"\"");
    props.push_back("}");
    std::string command = createQuery("CREATE (n:Vertex $props)", props);

    std::cout << command  << std::endl;

    web::http::http_response response = client->request(web::http::methods::POST, "/db/neo4j/tx/commit", command, "application/json;charset=utf-8").get();

    // Check the status code.
    if (response.status_code() != 200) {
        std::cout << "ERROR " << response.to_string() << std::endl;
        throw std::runtime_error("Returned " + std::to_string(response.status_code()));
    }
    // std::cout << response.to_string() << std::endl;
}

void Neo4jGraph::addEdge(const EdgeObject& e)
{
	boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);

    // CREATE (c1left:Sensor)-[:SE3]->(m1:Vertex)

    std::string command = createQuery("CREATE ("+std::to_string(e.source)+")-["+e.label+"]->("+std::to_string(e.target)+")");

    web::http::http_response response = client->request(web::http::methods::POST, "/db/neo4j/tx/commit", command, "application/json;charset=utf-8").get();

    // Check the status code.
    if (response.status_code() != 200) {
        std::cout << "ERROR " << response.to_string() << std::endl;
        throw std::runtime_error("Returned " + std::to_string(response.status_code()));
    }
}

void Neo4jGraph::removeEdge(IdType source, IdType target, const std::string& sensor)
{
	// boost::remove_edge(getEdgeIterator(source, target, sensor), mPoseGraph);
}

VertexObjectList Neo4jGraph::getVerticesFromSensor(const std::string& sensor) const
{
	VertexObjectList objectList;
	// VertexRange vertices = boost::vertices(mPoseGraph);
	// for(VertexIterator it = vertices.first; it != vertices.second; ++it)
	// {
	// 	if(mPoseGraph[*it].measurement->getSensorName() == sensor)
	// 	{
	// 		objectList.push_back(mPoseGraph[*it]);
	// 	}
	// }
	return objectList;
}

const VertexObject& Neo4jGraph::getVertex(IdType id)
{
    return getVertexInternal(id);
    // printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
    // vertexObjects.resize(id+1);
    // VertexObject& vertexobj = vertexObjects[id];

    //query and fill

    // std::string command = "CREATE ("+std::to_string(e.source)+")-["+e.label+"]->("+std::to_string(e.target)+")";

    // web::http::http_response response = client->request(web::http::methods::POST, "/db/neo4j/tx/commit", command, "application/json;charset=utf-8").get();

}

VertexObject& Neo4jGraph::getVertexInternal(IdType id)
{
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
    vertexObjects.resize(id+1);
    VertexObject& vertexobj = vertexObjects[id];

    //query and fill
    //MATCH (n:Vertex) WHERE n.index = "2" RETURN n AS node
    std::string command = createQuery("MATCH (n:Vertex) WHERE n.index = '"+std::to_string(id)+"' RETURN n.label AS label");

    std::cout << command << std::endl;

    web::http::http_response response = client->request(web::http::methods::POST, "/db/neo4j/tx/commit", command, "application/json;charset=utf-8").get();

    web::json::value reply = response.extract_json().get();

    std::cout << "reply: " << reply.serialize() << std::endl << std::endl;
    
    // todo check success // multiple replies
    vertexobj.index = id;
    vertexobj.label = reply["results"][0]["data"][0]["row"][0].as_string();

    std::cout << vertexobj.label << std::endl;
    
	return vertexobj;
}

const EdgeObject& Neo4jGraph::getEdge(IdType source, IdType target, const std::string& sensor) const
{
	// OutEdgeIterator it = getEdgeIterator(source, target, sensor);
	// return mPoseGraph[*it];
}

EdgeObject& Neo4jGraph::getEdgeInternal(IdType source, IdType target, const std::string& sensor)
{
	// OutEdgeIterator it = getEdgeIterator(source, target, sensor);
	// return mPoseGraph[*it];
}

// OutEdgeIterator Neo4jGraph::getEdgeIterator(IdType source, IdType target, const std::string& sensor) const
// {
// 	// OutEdgeIterator it, it_end;
// 	// try
// 	// {
// 	// 	boost::tie(it, it_end) = boost::out_edges(mIndexMap.at(source), mPoseGraph);
// 	// }catch(std::out_of_range &e)
// 	// {
// 	// 	throw InvalidVertex(source);
// 	// }
// 	// while(it != it_end)
// 	// {
// 	// 	const VertexObject& tObject = mPoseGraph[boost::target(*it, mPoseGraph)];
// 	// 	if(tObject.index == target && mPoseGraph[*it].constraint->getSensorName() == sensor)
// 	// 	{
// 	// 		return it;
// 	// 	}
// 	// 	++it;
// 	// }
// 	// throw InvalidEdge(source, target);
// }

EdgeObjectList Neo4jGraph::getOutEdges(IdType source) const
{
	// OutEdgeIterator it, it_end;
	// boost::tie(it, it_end) = boost::out_edges(mIndexMap.at(source), mPoseGraph);
	// EdgeObjectList edges;
	// while(it != it_end)
	// {
	// 	edges.push_back(mPoseGraph[*it]);
	// 	++it;
	// }
	// return edges;
}

EdgeObjectList Neo4jGraph::getEdges(const VertexObjectList& vertices) const
{
	// std::set<int> v_ids;
	// for(VertexObjectList::const_iterator v = vertices.begin(); v != vertices.end(); v++)
	// {
	// 	v_ids.insert(v->index);
	// }
	EdgeObjectList objectList;
	// EdgeRange edges = boost::edges(mPoseGraph);
	// for(EdgeIterator it = edges.first; it != edges.second; ++it)
	// {
	// 	EdgeObject ed = mPoseGraph[*it];
	// 	if(v_ids.find(ed.source) != v_ids.end() && v_ids.find(ed.target) != v_ids.end())
	// 		objectList.push_back(ed);
	// }
	return objectList;
}

void Neo4jGraph::writeGraphToFile(const std::string& name)
{
	// boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
	// std::string file = name + ".dot";
	// mLogger->message(INFO, (boost::format("Writing graph to file '%1%'.") % file).str());
	// std::ofstream ofs;
	// ofs.open(file.c_str());
	// boost::write_graphviz(
	// 		ofs,
	// 		mPoseGraph,
	// 		boost::make_label_writer(boost::get(&VertexObject::label, mPoseGraph)),
	// 		boost::make_label_writer(boost::get(&EdgeObject::label, mPoseGraph)),
	// 		boost::default_writer(),
	// 		boost::get(&VertexObject::index, mPoseGraph));
	// ofs.close();
}

// ================================================================
// BFS search for vertices with a maximum distance to a source node
// ================================================================

// struct EdgeFilter
// {
// 	EdgeFilter() {}
// 	EdgeFilter(const AdjacencyGraph* g) : graph(g) {}
// 	bool operator()(const Edge& e) const
// 	{
// 		return (*graph)[e].constraint->getType() == SE3;
// 	}
	
// 	const AdjacencyGraph* graph;
// };

// typedef boost::filtered_graph<AdjacencyGraph, EdgeFilter> FilteredGraph;
// typedef std::map<FilteredGraph::vertex_descriptor, boost::default_color_type> ColorMap;
// typedef std::map<FilteredGraph::vertex_descriptor, unsigned> DepthMap;

// /**
//  * @class MaxDepthVisitor
//  * @brief BFS-Visitor to find nearby nodes in the graph.
//  */
// class MaxDepthVisitor : public boost::default_bfs_visitor
// {
// public:
// 	MaxDepthVisitor(DepthMap& map, unsigned d) : depth_map(map), max_depth(d) {}

// 	void tree_edge(FilteredGraph::edge_descriptor e, const FilteredGraph& g)
// 	{
// 		FilteredGraph::vertex_descriptor u = source(e, g);
// 		FilteredGraph::vertex_descriptor v = target(e, g);
// 		if(depth_map[u] >= max_depth)
// 			throw 0;
// 		depth_map[v] = depth_map[u] + 1;
// 	}
// private:
// 	DepthMap& depth_map;
// 	unsigned max_depth;
// };

VertexObjectList Neo4jGraph::getVerticesInRange(IdType source_id, unsigned range) const
{
	// Create required data structures
	// Vertex source = mIndexMap.at(source_id);
	// DepthMap depth_map;
	// depth_map[source] = 0;
	// ColorMap c_map;
	// MaxDepthVisitor vis(depth_map, range);
	
	// // Do BFS on filtered graph
	// FilteredGraph fg(mPoseGraph, EdgeFilter(&mPoseGraph));
	// try
	// {
	// 	boost::breadth_first_search(fg, source, boost::visitor(vis).color_map(boost::associative_property_map<ColorMap>(c_map)));
	// }catch(int e)
	// {
	// }

	// Write the result
	VertexObjectList vertices;
	// for(DepthMap::iterator it = depth_map.begin(); it != depth_map.end(); ++it)
	// {
	// 	vertices.push_back(mPoseGraph[it->first]);
	// }
	return vertices;
}

float Neo4jGraph::calculateGraphDistance(IdType source_id, IdType target_id)
{
	// boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
	// int num = boost::num_vertices(mPoseGraph);
	// std::vector<Vertex> parent(num);
	// std::vector<float> distance(num);
	// std::map<Edge, float> weight;
	// EdgeRange edges = boost::edges(mPoseGraph);
	// EdgeFilter filter(&mPoseGraph);
	// for(EdgeIterator it = edges.first; it != edges.second; ++it)
	// {
	// 	if(filter(*it))
	// 		weight[*it] = 1.0;
	// 	else
	// 		weight[*it] = 10000;
	// }
	
	// boost::dijkstra_shortest_paths(mPoseGraph, mIndexMap.at(source_id),
	// 	boost::distance_map(boost::make_iterator_property_map(distance.begin(), boost::get(boost::vertex_index, mPoseGraph)))
	// 	.predecessor_map(boost::make_iterator_property_map(parent.begin(), boost::get(boost::vertex_index, mPoseGraph)))
	// 	.weight_map(boost::make_assoc_property_map(weight)) );

	// return distance[mIndexMap.at(target_id)];
	return 0;
}


std::string Neo4jGraph::createQuery(const std::string& query, const std::vector<std::string>& params) {
    std::string json = "{ \"statements\": [{\"statement\": \""+query+"\"";

    if (params.size()) {
        json += ", \"parameters\": {";
        for (const auto &param : params) {
        json += param;
        }
        json += "}";
    }
    json += "}]}";
    return json;

}
