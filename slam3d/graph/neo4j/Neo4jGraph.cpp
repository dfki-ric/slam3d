// workaround for:
//https://svn.boost.org/trac/boost/ticket/10382
// #define BOOST_NO_CXX11_DEFAULTED_FUNCTIONS


#include "Neo4jGraph.hpp"

// #include <boost/format.hpp>
#include <slam3d/core/Solver.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <fstream>


using namespace slam3d;

using namespace web;
using namespace http;
using namespace utility;
using status_codes = web::http::status_codes;
// using Client = web::http::client::http_client;

Neo4jGraph::Neo4jGraph(Logger* log, std::shared_ptr<MeasurementStorage> measurements, const Server &graphserver) : Graph(log), logger(log), measurements(measurements)
{
    web::http::client::http_client_config clientconf;
    clientconf.set_validate_certificates(false);
    web::credentials clientcred(_XPLATSTR("neo4j"), _XPLATSTR("neo4j"));
    clientconf.set_credentials(clientcred);
    std::cout << _XPLATSTR("http://"+graphserver.host+":" + std::to_string(graphserver.port)) << std::endl;
    client = std::make_shared<web::http::client::http_client>(_XPLATSTR("http://"+graphserver.host+":" + std::to_string(graphserver.port)), clientconf);

}

Neo4jGraph::~Neo4jGraph()
{
}

bool Neo4jGraph::deleteDatabase()
{
    Neo4jQuery vertexQuery(client);
    vertexQuery.addStatement("match (n) detach delete n");

    if (vertexQuery.sendQuery()) {
        return true;
    }
    return false;

}

const EdgeObjectList Neo4jGraph::getEdgesFromSensor(const std::string& sensor)  const
{
    EdgeObjectList objectList;

    Neo4jQuery query(client);
    query.addStatement("MATCH (a:Vertex)-[r]->(b:Vertex) WHERE r.sensor='"+sensor+"' AND r.inverted=false AND r.source <> r.target RETURN r");
    // query.addStatement("MATCH (a:Vertex)-[r]->(b:Vertex) WHERE r.sensor='"+sensor+"' RETURN r");

    if (!query.sendQuery()) {
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()));
    }
    web::json::value result = query.getResponse().extract_json().get();
    for (auto& jsonEdge : result["results"][0]["data"].as_array()) {
        objectList.push_back(Neo4jConversion::edgeObjectFromJson(jsonEdge["row"][0]));
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
    Neo4jQuery vertexQuery(client);
    vertexQuery.addStatement("CREATE (n:Vertex $props)");
    vertexQuery.addParameterSet("props");
    vertexQuery.addParameterToSet("props", "label", v.label);
    vertexQuery.addParameterToSet("props", "index", v.index);
    vertexQuery.addParameterToSet("props", "corrected_pose", Neo4jConversion::eigenMatrixToString(v.corrected_pose.matrix()));
    vertexQuery.addParameterToSet("props", "sensor", v.measurement->getSensorName());
    std::string uuid = boost::uuids::to_string(v.measurement->getUniqueId());
    vertexQuery.addParameterToSet("props", "measurement", uuid);

    measurements->set(uuid, v.measurement);

    // add position as extra statement (point property cannot be directly set via json props, there it is added as string)
    vertexQuery.addStatement("MATCH (n:Vertex) WHERE n.index="+std::to_string(v.index)+" SET n.location = point({"
                            +   "x: " + std::to_string(v.corrected_pose.translation().x())
                            + ", y: " + std::to_string(v.corrected_pose.translation().y())
                            + ", z: " + std::to_string(v.corrected_pose.translation().z())
                            + "})");

    if (!vertexQuery.sendQuery()) {
        logger->message(ERROR, vertexQuery.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(vertexQuery.getResponse().status_code()));
    }

}

void Neo4jGraph::addEdge(const EdgeObject& e) {
    addEdge(e, true);
}

void Neo4jGraph::addEdge(const EdgeObject& e, bool addInverse) {
    std::string constrainttypename = e.constraint->getTypeName();
    std::replace(constrainttypename.begin(), constrainttypename.end(), '(', '_');
    std::replace(constrainttypename.begin(), constrainttypename.end(), ')', '_');

    Neo4jQuery query(client);
    query.addStatement("MATCH (a:Vertex), (b:Vertex) WHERE a.index="+std::to_string(e.source)+" AND b.index="+std::to_string(e.target) \
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
    Neo4jConversion::constraintToJson(e.constraint, query.getParameterSet("props"));

    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }

    // query.printQuery();

    if (addInverse) {
        // reverse edge TODO: need inverse transform/identification?
        query.setStatement(0, "MATCH (a:Vertex), (b:Vertex) WHERE a.index="+std::to_string(e.target)+" AND b.index="+std::to_string(e.source) \
            + " CREATE (a)-[r:" + constrainttypename + " $props]->(b) RETURN type(r)");
        query.addParameterToSet("props", "inverted", true);
        query.addParameterToSet("props", "source", e.target);
        query.addParameterToSet("props", "target", e.source);

        if (!query.sendQuery()) {
            logger->message(ERROR, query.getResponse().extract_string().get());
            throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
        }
    }
}

void Neo4jGraph::removeEdge(IdType source, IdType target, const std::string& sensor) {
    Neo4jQuery query(client);
    // MATCH (n:Person {name: 'Laurence Fishburne'})-[r:ACTED_IN]->() DELETE r
    query.addStatement("MATCH ("+std::to_string(source)+")-[r]->("+std::to_string(target)+") DELETE r");
    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }
}

const VertexObjectList Neo4jGraph::getVerticesFromSensor(const std::string& sensor)  const{
    VertexObjectList objectList;

    Neo4jQuery query(client);
    query.addStatement("MATCH (a:Vertex) WHERE a.sensor='"+sensor+"' RETURN a");

    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()));
    }
    web::json::value result = query.getResponse().extract_json().get();

    for (auto& jsonEdge : result["results"][0]["data"].as_array()) {
        slam3d::VertexObject vertex = Neo4jConversion::vertexObjectFromJson(jsonEdge["row"][0], measurements);
        objectList.push_back(vertex);
    }
    return objectList;
}

const VertexObject Neo4jGraph::getVertex(IdType id)  const{
    std::lock_guard<std::mutex> lock (queryMutex);
    VertexObject vertexobj;

    //query and fill
    //MATCH (n:Vertex) WHERE n.index = "2" RETURN n AS node
    Neo4jQuery query(client);
    // MATCH (n:Person {name: 'Laurence Fishburne'})-[r:ACTED_IN]->() DELETE r
    query.addStatement("MATCH (n:Vertex) WHERE n.index = "+std::to_string(id)+" RETURN n AS node");
    if (!query.sendQuery()) {
        std::string msg = query.getResponse().extract_string().get();
        logger->message(ERROR, msg);
        std::cout << msg << std::endl;
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }
    web::json::value reply = query.getResponse().extract_json().get();

    // printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
    // std::cout << reply["results"][0]["data"][0]["row"][0].serialize() << std::endl;

    vertexobj = Neo4jConversion::vertexObjectFromJson(reply["results"][0]["data"][0]["row"][0], measurements);

    return vertexobj;
}

void Neo4jGraph::setVertex(IdType id, const VertexObject& v) {
    // std::lock_guard<std::mutex> lock (queryMutex);

    Neo4jQuery vertexQuery(client);

    // add position as extra statement (point property cannot be directly set via json props, there it is added as string)
    vertexQuery.addStatement("MATCH (n:Vertex) WHERE n.index="+std::to_string(v.index)+" SET n = $props");
    
    vertexQuery.addParameterSet("props");
    vertexQuery.addParameterToSet("props", "label", v.label);
    vertexQuery.addParameterToSet("props", "index", v.index);
    vertexQuery.addParameterToSet("props", "corrected_pose", Neo4jConversion::eigenMatrixToString(v.corrected_pose.matrix()));
    vertexQuery.addParameterToSet("props", "sensor", v.measurement->getSensorName());
    std::string uuid = boost::uuids::to_string(v.measurement->getUniqueId());
    vertexQuery.addParameterToSet("props", "measurement", uuid);

    // replace measurement in reg
    if (v.measurement.get() != nullptr) {
        measurements->set(uuid, v.measurement);
    }

    if (!vertexQuery.sendQuery()) {
        logger->message(ERROR, vertexQuery.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(vertexQuery.getResponse().status_code()));
    }
}

const EdgeObject Neo4jGraph::getEdge(IdType source, IdType target, const std::string& sensor) const {
    Neo4jQuery query(client);
    // MATCH (a:Vertex)-[r]->(b:Vertex) WHERE a.index=1 AND b.index=2 AND r.sensor="S1" RETURN r
    query.addStatement("MATCH (a:Vertex)-[r]->(b:Vertex) WHERE a.index="+std::to_string(source)+" AND b.index="+std::to_string(target)+" AND r.sensor='"+sensor+"' RETURN r");
    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }

    web::json::value reply = query.getResponse().extract_json().get();
    if (reply["results"][0]["data"].size() == 0) {
        throw InvalidEdge(source, target);
    }

    // TODO: danger! returning reference to this, not threadsafe! NOT save for calling multiple times before receiving
    static EdgeObject returnval;
    returnval = Neo4jConversion::edgeObjectFromJson(reply["results"][0]["data"][0]["row"][0]);

    return returnval;
}

const EdgeObjectList Neo4jGraph::getOutEdges(IdType source) const
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

const EdgeObjectList Neo4jGraph::getEdges(const VertexObjectList& vertices) const
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
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
}

const VertexObjectList Neo4jGraph::getVerticesInRange(IdType source_id, unsigned range) const
{
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);

    // TODO:

    Neo4jQuery query(client);

    query.addStatement("match (v1:Vertex), (v2:Vertex) where v1.index="+std::to_string(source_id)+" AND point.distance(v1.location, v2.location) <= "+std::to_string(range)+" return v2");

    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }

    web::json::value result = query.getResponse().extract_json().get();
    try {
        web::json::array &nodes = result["results"][0]["data"][0]["row"][0].as_array();
        
    } catch (const web::json::json_exception &e) {
        std::cout << "error on getVerticesInRange : " << source_id << " -- " << e.what() << std::endl;
        std::cout << result << std::endl;
    }

    // Write the result
    VertexObjectList vertices;
    // for(DepthMap::iterator it = depth_map.begin(); it != depth_map.end(); ++it)
    // {
    // 	vertices.push_back(mPoseGraph[it->first]);
    // }
    return vertices;
}

float Neo4jGraph::calculateGraphDistance(IdType source_id, IdType target_id) const {
    Neo4jQuery query(client);
    // MATCH (a:Vertex), (b:Vertex), p=shortestPath((a)-[*]-(b)) WHERE a.index = 1 AND b.index = 28 return p
    query.addStatement("MATCH (a:Vertex), (b:Vertex), p=shortestPath((a)-[*]->(b)) WHERE a.index="+std::to_string(source_id)+" AND b.index="+std::to_string(target_id)+" RETURN RELATIONSHIPS(p)");

    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }

    web::json::value result = query.getResponse().extract_json().get();
    try {
        web::json::array &path = result["results"][0]["data"][0]["row"][0].as_array();
        return path.size();
    } catch (const web::json::json_exception &e) {
        std::cout << "error on calculateGraphDistance : " << source_id << " -- " << target_id << e.what() << std::endl;
        std::cout << result << std::endl;
    }
    return 0;
}


void Neo4jGraph::setCorrectedPose(IdType id, const Transform& pose)
{
    Neo4jQuery query(client);
    // add position as extra statement (point property cannot be directly set via json props, there it is added as string)
    query.addStatement("MATCH (n:Vertex) WHERE n.index="+std::to_string(id)+" SET n.location = point({"
                            +   "x: " + std::to_string(pose.translation().x())
                            + ", y: " + std::to_string(pose.translation().y())
                            + ", z: " + std::to_string(pose.translation().z())
                            + "}) ,"
                            + " n.corrected_pose = \"" + Neo4jConversion::eigenMatrixToString(pose.matrix()) + "\"");

    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }

}

