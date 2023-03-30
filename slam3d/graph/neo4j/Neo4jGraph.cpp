// workaround for:
//https://svn.boost.org/trac/boost/ticket/10382
// #define BOOST_NO_CXX11_DEFAULTED_FUNCTIONS


#include "Neo4jGraph.hpp"

// #include <boost/format.hpp>
#include <slam3d/core/Solver.hpp>

#include <fstream>


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
    web::credentials clientcred(_XPLATSTR("neo4j"), _XPLATSTR("neo4j"));
    clientconf.set_credentials(clientcred);
    client = std::make_shared<web::http::client::http_client>(_XPLATSTR("http://localhost:7474"), clientconf);

}

Neo4jGraph::~Neo4jGraph()
{
    // neo4j_close(connection);
    // neo4j_client_cleanup();
}

bool Neo4jGraph::deleteDatabase() {

    Query vertexQuery(client);
    vertexQuery.setStatement("match (n) detach delete n");

    if (vertexQuery.sendQuery()) {
        return true;
    }
    return false;

}

EdgeObjectList Neo4jGraph::getEdgesFromSensor(const std::string& sensor)
{
    EdgeObjectList objectList;

    Query query(client);
    query.setStatement("MATCH (a:Vertex)-[r]->(b:Vertex) WHERE r.sensor='"+sensor+"' AND r.inverted=false RETURN r");
    // query.setStatement("MATCH (a:Vertex)-[r]->(b:Vertex) WHERE r.sensor='"+sensor+"' RETURN r");

    if (!query.sendQuery()) {
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()));
    }
    web::json::value result = query.getResponse().extract_json().get();
    for (auto& jsonEdge : result["results"][0]["data"].as_array()) {
        objectList.push_back(edgeObjectFromJson(jsonEdge["row"][0]));
    }

    return objectList;
}

bool Neo4jGraph::optimize(unsigned iterations)
{
    // boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
    return Graph::optimize(iterations);
}

void Neo4jGraph::addVertex(const VertexObject& v)
{
    Query vertexQuery(client);
    vertexQuery.setStatement("CREATE (n:Vertex $props)");
    vertexQuery.addParameterSet("props");
    vertexQuery.addParameterToSet("props", "label", v.label);
    vertexQuery.addParameterToSet("props", "index", v.index);
    vertexQuery.addParameterToSet("props", "mRelativePose", eigenMatrixToString(Transform::Identity().matrix()));
    vertexQuery.addParameterToSet("props", "measurement", "uuid?");

    if (!vertexQuery.sendQuery()) {
        throw std::runtime_error("Returned " + std::to_string(vertexQuery.getResponse().status_code()));
    }
}

void Neo4jGraph::addEdge(const EdgeObject& e) {
    std::string constrainttypename = e.constraint->getTypeName();
    std::replace(constrainttypename.begin(), constrainttypename.end(), '(', '_');
    std::replace(constrainttypename.begin(), constrainttypename.end(), ')', '_');

    Query query(client);
    query.setStatement("MATCH (a:Vertex), (b:Vertex) WHERE a.index="+std::to_string(e.source)+" AND b.index="+std::to_string(e.target) \
        + " CREATE (a)-[r:" + constrainttypename + " $props]->(b) RETURN type(r)");
    query.addParameterSet("props");
    query.addParameterToSet("props", "label", e.label);
    query.addParameterToSet("props", "source", e.source);
    query.addParameterToSet("props", "target", e.target);
    query.addParameterToSet("props", "sensor", e.constraint->getSensorName());
    query.addParameterToSet("props", "timestamp_tv_sec", e.constraint->getTimestamp().tv_sec);
    query.addParameterToSet("props", "timestamp_tv_usec", e.constraint->getTimestamp().tv_usec);
    query.addParameterToSet("props", "inverted", false);
    query.addParameterToSet("props", "constraint_type", e.constraint->getType());
    constraintToJson(e.constraint, query.getParameterSet("props"));

    if (!query.sendQuery()) {
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }

    // reverse edge TODO: need inverse transform/identification?
    query.setStatement("MATCH (a:Vertex), (b:Vertex) WHERE a.index="+std::to_string(e.target)+" AND b.index="+std::to_string(e.source) \
        + " CREATE (a)-[r:" + constrainttypename + " $props]->(b) RETURN type(r)");
    query.addParameterToSet("props", "inverted", true);
    query.addParameterToSet("props", "source", e.target);
    query.addParameterToSet("props", "target", e.source);

    if (!query.sendQuery()) {
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }
}

void Neo4jGraph::removeEdge(IdType source, IdType target, const std::string& sensor) {
    Query query(client);
    // MATCH (n:Person {name: 'Laurence Fishburne'})-[r:ACTED_IN]->() DELETE r
    query.setStatement("MATCH ("+std::to_string(source)+")-[r]->("+std::to_string(target)+") DELETE r");
    if (!query.sendQuery()) {
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }
}

slam3d::VertexObject Neo4jGraph::vertexObjectFromJson(web::json::value& json) {
    slam3d::VertexObject returnval;
    returnval.index = json["index"].as_integer();
    returnval.label = json["label"].as_string();
    returnval.corrected_pose = Eigen::Matrix4d(eigenMatrixFromString(json["mRelativePose"].as_string()));
    //TODO: fill with data
    returnval.measurement = slam3d::Measurement::Ptr();

    return returnval;
}

VertexObjectList Neo4jGraph::getVerticesFromSensor(const std::string& sensor) {
    VertexObjectList objectList;


    Query query(client);
    query.setStatement("MATCH (a:Vertex)-[r]->(b:Vertex) WHERE r.sensor='"+sensor+"' AND r.inverted=false RETURN a AS source, b AS target");

    if (!query.sendQuery()) {
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()));
    }
    web::json::value result = query.getResponse().extract_json().get();

    for (auto& jsonEdge : result["results"][0]["data"].as_array()) {
        if (objectList.empty()) {
            // on the first result, also ass source, only target elsewise
            objectList.push_back(vertexObjectFromJson(jsonEdge["row"][0]));
        }
        objectList.push_back(vertexObjectFromJson(jsonEdge["row"][1]));
    }
    return objectList;
}

const VertexObject& Neo4jGraph::getVertex(IdType id)
{
    return getVertexInternal(id);
}

VertexObject& Neo4jGraph::getVertexInternal(IdType id)
{
    vertexObjects.resize(id+1);
    VertexObject& vertexobj = vertexObjects[id];

    //query and fill
    //MATCH (n:Vertex) WHERE n.index = "2" RETURN n AS node
    Query query(client);
    // MATCH (n:Person {name: 'Laurence Fishburne'})-[r:ACTED_IN]->() DELETE r
    query.setStatement("MATCH (n:Vertex) WHERE n.index = "+std::to_string(id)+" RETURN n AS node");
    if (!query.sendQuery()) {
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }
    web::json::value reply = query.getResponse().extract_json().get();

    vertexobj = vertexObjectFromJson(reply["results"][0]["data"][0]["row"][0]);
    return vertexobj;
}

const EdgeObject& Neo4jGraph::getEdge(IdType source, IdType target, const std::string& sensor)
{
    return getEdgeInternal(source, target, sensor);
}

slam3d::EdgeObject Neo4jGraph::edgeObjectFromJson(web::json::value& json) {
    slam3d::EdgeObject returnval;
    if (json["inverted"].as_bool()) {
        returnval.source = json["target"].as_integer();
        returnval.target = json["source"].as_integer();
    } else {
        returnval.source = json["source"].as_integer();
        returnval.target = json["target"].as_integer();
    }
    //TODO: Serialize/recreate all contstraints with content
    returnval.constraint = jsonToConstraint(json);
    return returnval;
}

EdgeObject& Neo4jGraph::getEdgeInternal(IdType source, IdType target, const std::string& sensor) {
    Query query(client);
    // MATCH (a:Vertex)-[r]->(b:Vertex) WHERE a.index=1 AND b.index=2 AND r.sensor="S1" RETURN r
    query.setStatement("MATCH (a:Vertex)-[r]->(b:Vertex) WHERE a.index="+std::to_string(source)+" AND b.index="+std::to_string(target)+" AND r.sensor='"+sensor+"' RETURN r");
    if (!query.sendQuery()) {
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }

    web::json::value reply = query.getResponse().extract_json().get();
    if (reply["results"][0]["data"].size() == 0) {
        throw InvalidEdge(source, target);
    }

    // TODO: danger! returning reference to this, not threadsafe! NOT save for calling multiple times before receiving
    static EdgeObject returnval;
    returnval = edgeObjectFromJson(reply["results"][0]["data"][0]["row"][0]);

    return returnval;
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
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
    //OutEdgeIterator it, it_end;
    // boost::tie(it, it_end) = boost::out_edges(mIndexMap.at(source), mPoseGraph);
    EdgeObjectList edges;
    // while(it != it_end)
    // {
    // 	edges.push_back(mPoseGraph[*it]);
    // 	++it;
    // }
    return edges;
}

EdgeObjectList Neo4jGraph::getEdges(const VertexObjectList& vertices) const
{
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);

    // TODO:

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
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);

    // TODO:

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
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
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


std::string Neo4jGraph::createQuery(const std::string& query, const web::json::value& params) {
    std::string json = "{ \"statements\": [{\"statement\": \""+query+"\"";

    json += ", \"parameters\": ";
    if (!params.is_null()) {
        json += params.serialize();
    } else {
        json += "{}";
    }


    json += "}]}";
    return json;

}


void Neo4jGraph::constraintToJson(slam3d::Constraint::Ptr constraint, web::json::value* json) {
    web::json::value& data = *json;

    switch (constraint->getType()) {
        case slam3d::TENTATIVE : break;
        case slam3d::SE3 : {
            slam3d::SE3Constraint* se3 = dynamic_cast<slam3d::SE3Constraint*>(constraint.get());
            data["mRelativePose"] = web::json::value(eigenMatrixToString(se3->getRelativePose().matrix()));
            data["mInformation"] = web::json::value(eigenMatrixToString(se3->getInformation().matrix()));
            break;
        }
        case slam3d::GRAVITY : {
            slam3d::GravityConstraint* grav = dynamic_cast<slam3d::GravityConstraint*>(constraint.get());
            data["mDirection"] = web::json::value(eigenMatrixToString(grav->getDirection().matrix()));
            data["mReference"] = web::json::value(eigenMatrixToString(grav->getReference().matrix()));
            data["mCovariance"] = web::json::value(eigenMatrixToString(grav->getCovariance().matrix()));
            break;
        }
        case slam3d::POSITION : {
            slam3d::PositionConstraint* grav = dynamic_cast<slam3d::PositionConstraint*>(constraint.get());
            data["mPosition"] = web::json::value(eigenMatrixToString(grav->getPosition().matrix()));
            data["mSensorPose"] = web::json::value(eigenMatrixToString(grav->getSensorPose().matrix()));
            data["mCovariance"] = web::json::value(eigenMatrixToString(grav->getCovariance().matrix()));
            break;
        }
        case slam3d::ORIENTATION : {
            slam3d::OrientationConstraint* grav = dynamic_cast<slam3d::OrientationConstraint*>(constraint.get());
            data["mOrientation"] = web::json::value(eigenMatrixToString(grav->getOrientation().matrix()));
            data["mSensorPose"] = web::json::value(eigenMatrixToString(grav->getSensorPose().matrix()));
            data["mCovariance"] = web::json::value(eigenMatrixToString(grav->getCovariance().matrix()));
            break;
        }
    }
}

slam3d::Constraint::Ptr Neo4jGraph::jsonToConstraint(web::json::value& json) {
    switch (json["constraint_type"].as_integer()) {
        case slam3d::TENTATIVE : return slam3d::Constraint::Ptr(new slam3d::TentativeConstraint(json["sensor"].as_string()));
        case slam3d::SE3 : {
            slam3d::Transform t = slam3d::Transform(Eigen::Matrix4d(eigenMatrixFromString(json["mRelativePose"].as_string())));
            slam3d::Covariance<6> i = slam3d::Covariance<6>(eigenMatrixFromString(json["mInformation"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::SE3Constraint(json["sensor"].as_string(), t, i));
        }
        case slam3d::GRAVITY : {
            slam3d::Direction d = slam3d::Direction(eigenMatrixFromString(json["mDirection"].as_string()));
            slam3d::Direction r = slam3d::Direction(eigenMatrixFromString(json["mReference"].as_string()));
            slam3d::Covariance<2> c = slam3d::Covariance<2>(eigenMatrixFromString(json["mCovariance"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::GravityConstraint(json["sensor"].as_string(), d, r, c));
        }
        case slam3d::POSITION : {
            slam3d::Position p = slam3d::Position(eigenMatrixFromString(json["mPosition"].as_string()));
            slam3d::Transform t = slam3d::Transform(Eigen::Matrix4d(eigenMatrixFromString(json["mSensorPose"].as_string())));
            slam3d::Covariance<3> c = slam3d::Covariance<3>(eigenMatrixFromString(json["mCovariance"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::PositionConstraint(json["sensor"].as_string(), p, c, t));
        }
        case slam3d::ORIENTATION : {
            slam3d::Quaternion q = slam3d::Quaternion(Eigen::Matrix3d(eigenMatrixFromString(json["mOrientation"].as_string())));
            slam3d::Transform t = slam3d::Transform(Eigen::Matrix4d(eigenMatrixFromString(json["mSensorPose"].as_string())));
            slam3d::Covariance<3> c = slam3d::Covariance<3>(eigenMatrixFromString(json["mCovariance"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::OrientationConstraint(json["sensor"].as_string(), q, c, t));
        }
    }
    // should never be here
    return slam3d::Constraint::Ptr();
}


std::string Neo4jGraph::eigenMatrixToString(const Eigen::MatrixXd& mat) {
    std::stringstream ss;
    // the '\n' id not needed but makes the value more readable in the neo4j browser
    Eigen::IOFormat jsonfmt(Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");
    ss << mat.format(jsonfmt);
    return ss.str();
}

Eigen::MatrixXd Neo4jGraph::eigenMatrixFromString(const std::string & string) {
    web::json::value val = web::json::value::parse(string);
    Eigen::MatrixXd mat(val.size(), val[0].size());
    for (size_t y = 0; y < val[0].size(); ++y) {
        for (size_t x = 0; x < val.size(); ++x) {
            mat(x, y) = val[x][y].as_double();
        }
    }
    return mat;
}