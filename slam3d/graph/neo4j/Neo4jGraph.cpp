// workaround for:
//https://svn.boost.org/trac/boost/ticket/10382
// #define BOOST_NO_CXX11_DEFAULTED_FUNCTIONS


#include "Neo4jGraph.hpp"

// #include <boost/format.hpp>
#include <slam3d/core/Solver.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

#include <fstream>



using namespace slam3d;

using namespace web;
using namespace http;
using namespace utility;
using status_codes = web::http::status_codes;
// using Client = web::http::client::http_client;

Neo4jGraph::Neo4jGraph(Logger* log, MeasurementStorage* storage, const Server &graphserver) : Graph(log, storage), logger(log)
{
    web::http::client::http_client_config clientconf;
    clientconf.set_validate_certificates(false);
    web::credentials clientcred(_XPLATSTR("neo4j"), _XPLATSTR("neo4j"));
    clientconf.set_credentials(clientcred);
    std::cout << _XPLATSTR("http://"+graphserver.host+":" + std::to_string(graphserver.port)) << std::endl;
    client = std::make_shared<web::http::client::http_client>(_XPLATSTR("http://"+graphserver.host+":" + std::to_string(graphserver.port)), clientconf);

    connection = neo4j_connect("neo4j://neo4j:neo4j@localhost:7687", NULL, NEO4J_INSECURE);

}

Neo4jGraph::~Neo4jGraph()
{
    neo4j_close(connection);
    neo4j_client_cleanup();
}

bool Neo4jGraph::deleteDatabase()
{
    std::lock_guard<std::mutex> lock(queryMutex);
    Neo4jQuery vertexQuery(client);
    vertexQuery.addStatement("match (n) detach delete n");

    if (vertexQuery.sendQuery()) {
        return true;
    }
    return false;

}

const EdgeObjectList Neo4jGraph::getEdgesFromSensor(const std::string& sensor)  const
{
    std::lock_guard<std::mutex> lock(queryMutex);
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
    std::lock_guard<std::mutex> lock(queryMutex);
    Neo4jQuery vertexQuery(client);

    vertexQuery.addStatement("CREATE (n:Vertex $props)");
    vertexQuery.addParameterSet("props");
    vertexQuery.addParameterToSet("props", "label", v.label);
    vertexQuery.addParameterToSet("props", "index", v.index);
    vertexQuery.addParameterToSet("props", "correctedPose", Neo4jConversion::eigenMatrixToString(v.correctedPose.matrix()));
    vertexQuery.addParameterToSet("props", "robotName", v.robotName);
    vertexQuery.addParameterToSet("props", "sensorName", v.sensorName);
    vertexQuery.addParameterToSet("props", "typeName", v.typeName);
    vertexQuery.addParameterToSet("props", "timestamp_tv_sec", v.timestamp.tv_sec);
    vertexQuery.addParameterToSet("props", "timestamp_tv_usec", v.timestamp.tv_usec);

    std::string uuid = boost::lexical_cast<std::string>(v.measurementUuid);
    vertexQuery.addParameterToSet("props", "measurementUuid", uuid);

    // mStorage->add(measurement);

    // add position as extra statement (point property cannot be directly set via json props, there it is added as string)
    vertexQuery.addStatement("MATCH (n:Vertex) WHERE n.index="+std::to_string(v.index)+" SET n.location = point({"
                            +   "x: " + std::to_string(v.correctedPose.translation().x())
                            + ", y: " + std::to_string(v.correctedPose.translation().y())
                            + ", z: " + std::to_string(v.correctedPose.translation().z())
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
    std::lock_guard<std::mutex> lock(queryMutex);
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
    std::lock_guard<std::mutex> lock(queryMutex);
    Neo4jQuery query(client);
    // MATCH (n:Person {name: 'Laurence Fishburne'})-[r:ACTED_IN]->() DELETE r 
    //query.addStatement("MATCH (a:Vertex"+std::to_string(source)+")-[r]->("+std::to_string(target)+") DELETE r");
    query.addStatement("MATCH (a:Vertex)-[r]-(b:Vertex) WHERE a.index="+std::to_string(source)+" AND b.index="+std::to_string(target)+" AND r.sensor=\""+sensor+"\" DELETE r");
    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }
}


const std::set<std::string> Neo4jGraph::getVertexSensors() const {
    std::set<std::string> result;
    std::lock_guard<std::mutex> lock(queryMutex);
    Neo4jQuery query(client);
    query.addStatement("MATCH (a:Vertex) RETURN DISTINCT a.sensorName");

    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()));
    }
    web::json::value reply = query.getResponse().extract_json().get();
    //std::cout << reply.serialize() << std::endl;
    for (auto& json : reply["results"][0]["data"].as_array()) {
        result.insert(json["row"][0].as_string());
    }
    return result;
}

const std::set<std::string> Neo4jGraph::getEdgeSensors() const {
    std::set<std::string> result;
    std::lock_guard<std::mutex> lock(queryMutex);
    Neo4jQuery query(client);
    query.addStatement("MATCH ()-[r]->() RETURN DISTINCT r.sensor");

    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()));
    }
    web::json::value reply = query.getResponse().extract_json().get();
    // std::cout << reply.serialize() << std::endl;
    for (auto& json : reply["results"][0]["data"].as_array()) {
        result.insert(json["row"][0].as_string());
    }

    return result;
}


const VertexObjectList Neo4jGraph::getVerticesFromSensor(const std::string& sensor)  const{
    std::lock_guard<std::mutex> lock(queryMutex);
    VertexObjectList objectList;

    Neo4jQuery query(client);
    query.addStatement("MATCH (a:Vertex) WHERE a.sensorName='"+sensor+"' RETURN a");

    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()));
    }
    web::json::value result = query.getResponse().extract_json().get();

    for (auto& jsonEdge : result["results"][0]["data"].as_array()) {
        slam3d::VertexObject vertex = Neo4jConversion::vertexObjectFromJson(jsonEdge["row"][0]);
        objectList.push_back(vertex);
    }
    return objectList;
}

const VertexObjectList Neo4jGraph::getVerticesByType(const std::string& type) const {
    std::lock_guard<std::mutex> lock(queryMutex);
    VertexObjectList objectList;

    Neo4jQuery query(client);
    query.addStatement("MATCH (a:Vertex) WHERE a.typeName='"+type+"' RETURN a");

    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()));
    }
    web::json::value result = query.getResponse().extract_json().get();

    for (auto& jsonEdge : result["results"][0]["data"].as_array()) {
        slam3d::VertexObject vertex = Neo4jConversion::vertexObjectFromJson(jsonEdge["row"][0]);
        objectList.push_back(vertex);
    }
    return objectList;
}

const VertexObjectList Neo4jGraph::getNearbyVertices(const Transform &location, float radius, const std::string& sensortype) const {
    std::lock_guard<std::mutex> lock(queryMutex);
    VertexObjectList objectList;

    Neo4jQuery query(client);
    //match (n) where point.distance(point({x:-20, y:10, z:0}), n.location) < 15 return n
    if (sensortype == "") {
        query.addStatement("MATCH (a:Vertex) WHERE point.distance(point({x:"+std::to_string(location.translation().x())+", y:"+std::to_string(location.translation().y())+", z:"+std::to_string(location.translation().z())+"}), a.location) < "+std::to_string(radius)+" RETURN a");
    } else {
        query.addStatement("MATCH (a:Vertex) WHERE point.distance(point({x:"+std::to_string(location.translation().x())+", y:"+std::to_string(location.translation().y())+", z:"+std::to_string(location.translation().z())+"}), a.location) < "+std::to_string(radius)+" AND a.typeName = \""+sensortype+"\" RETURN a");
    }

    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()));
    }
    web::json::value result = query.getResponse().extract_json().get();

    for (auto& jsonEdge : result["results"][0]["data"].as_array()) {
        slam3d::VertexObject vertex = Neo4jConversion::vertexObjectFromJson(jsonEdge["row"][0]);
        objectList.push_back(vertex);
    }
    return objectList;
}


const VertexObject Neo4jGraph::getVertex(IdType id)  const {
    std::lock_guard<std::mutex> lock(queryMutex);
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
    try {
        vertexobj = Neo4jConversion::vertexObjectFromJson(reply["results"][0]["data"][0]["row"][0]);
    } catch (const web::json::json_exception& e) {
        printf("%s:%i %s\n", __PRETTY_FUNCTION__, __LINE__,e.what());
        vertexobj = VertexObject();
    }

    return vertexobj;
}

const VertexObject Neo4jGraph::getVertex(boost::uuids::uuid id) const {
    std::lock_guard<std::mutex> lock(queryMutex);
    VertexObject vertexobj;

    //query and fill
    //MATCH (n:Vertex) WHERE n.index = "2" RETURN n AS node
    Neo4jQuery query(client);
    // MATCH (n:Person {name: 'Laurence Fishburne'})-[r:ACTED_IN]->() DELETE r
    std::string uuid = boost::lexical_cast<std::string>(id);
    query.addStatement("MATCH (n:Vertex) WHERE n.measurementUuid = "+uuid+" RETURN n AS node");
    if (!query.sendQuery()) {
        std::string msg = query.getResponse().extract_string().get();
        logger->message(ERROR, msg);
        std::cout << msg << std::endl;
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }
    web::json::value reply = query.getResponse().extract_json().get();

    // printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
    // std::cout << reply["results"][0]["data"][0]["row"][0].serialize() << std::endl;

    vertexobj = Neo4jConversion::vertexObjectFromJson(reply["results"][0]["data"][0]["row"][0]);

    return vertexobj;
}

void Neo4jGraph::setVertex(IdType id, const VertexObject& v) {
    std::lock_guard<std::mutex> lock (queryMutex);

    Neo4jQuery vertexQuery(client);

    // add position as extra statement (point property cannot be directly set via json props, there it is added as string)
    vertexQuery.addStatement("MATCH (n:Vertex) WHERE n.index="+std::to_string(v.index)+" SET n = $props");
    vertexQuery.addParameterSet("props");
    vertexQuery.addParameterToSet("props", "label", v.label);
    vertexQuery.addParameterToSet("props", "index", v.index);
    vertexQuery.addParameterToSet("props", "correctedPose", Neo4jConversion::eigenMatrixToString(v.correctedPose.matrix()));
    vertexQuery.addParameterToSet("props", "sensorName", v.sensorName);
    vertexQuery.addParameterToSet("props", "robotName", v.robotName);
    vertexQuery.addParameterToSet("props", "typeName", v.typeName);
    vertexQuery.addParameterToSet("props", "timestamp_tv_sec", v.timestamp.tv_sec);
    vertexQuery.addParameterToSet("props", "timestamp_tv_usec", v.timestamp.tv_usec);

    std::string uuid = boost::uuids::to_string(v.measurementUuid);
    vertexQuery.addParameterToSet("props", "measurementUuid", uuid);

    if (!vertexQuery.sendQuery()) {
        logger->message(ERROR, vertexQuery.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(vertexQuery.getResponse().status_code()));
    }
}

const EdgeObject Neo4jGraph::getEdge(IdType source, IdType target, const std::string& sensor) const {
    std::lock_guard<std::mutex> lock(queryMutex);
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

const EdgeObjectList Neo4jGraph::getOutEdges(IdType source) const {
    std::lock_guard<std::mutex> lock(queryMutex);
    EdgeObjectList edgeObjectList;
    //match (n)-[r]->() where n.index=1 return r
    Neo4jQuery query(client);
    query.addStatement("MATCH (a:Vertex)-[r]->() WHERE a.index="+std::to_string(source)+" RETURN r");
    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }

    web::json::value reply = query.getResponse().extract_json().get();
    web::json::array& results = reply["results"][0]["data"].as_array();
    edgeObjectList.reserve(results.size());   
    
    for (auto& edge : results) {
        EdgeObject edgeobj;
        edgeobj = Neo4jConversion::edgeObjectFromJson(edge["row"][0]);
        edgeObjectList.push_back(edgeobj);
    }

    return edgeObjectList;
}

const EdgeObjectList Neo4jGraph::getEdges(const VertexObjectList& vertices) const {
    std::lock_guard<std::mutex> lock(queryMutex);
    EdgeObjectList edgeObjectList;
    //match ()-[r]->() return r
    Neo4jQuery query(client);
    query.addStatement("MATCH ()-[r]->() RETURN r");
    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }

    web::json::value reply = query.getResponse().extract_json().get();
    web::json::array& results = reply["results"][0]["data"].as_array();
    edgeObjectList.reserve(results.size());   
    
    for (auto& edge : results) {
        EdgeObject edgeobj;
        edgeobj = Neo4jConversion::edgeObjectFromJson(edge["row"][0]);
        edgeObjectList.push_back(edgeobj);
    }

    return edgeObjectList;
}

void Neo4jGraph::writeGraphToFile(const std::string& name)
{
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
}

const VertexObjectList Neo4jGraph::getVerticesInRange(IdType source_id, unsigned range) const
{
    std::lock_guard<std::mutex> lock(queryMutex);
    VertexObjectList vertexobjlist;
    
    Neo4jQuery query(client);
    query.addStatement("match (v1:Vertex)--{1,"+std::to_string(range)+"}(v2:Vertex) where v1.index="+std::to_string(source_id)+" return v2 as node");

    if (!query.sendQuery()) {
        std::string msg = query.getResponse().extract_string().get();
        logger->message(ERROR, msg);
        std::cout << msg << std::endl;
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }

    web::json::value reply = query.getResponse().extract_json().get();
    if (reply["results"].size()) {
        web::json::array& results = reply["results"][0]["data"].as_array();
        vertexobjlist.reserve(results.size());
        for (auto& vertex : results) {
            VertexObject vertexobj;
            vertexobj = Neo4jConversion::vertexObjectFromJson(vertex["row"][0]);
            vertexobjlist.push_back(vertexobj);
        }
    }
    return vertexobjlist;
}

const VertexObjectList Neo4jGraph::getAllVertices() const {


    slam3d::VertexObjectList vertexobjlist;
    neo4j_result_stream_t *results = neo4j_run(connection, "MATCH (n:Vertex) RETURN n AS node", neo4j_null);
    neo4j_result_t *result = neo4j_fetch_next(results);
    // std::map<std::string> nodemap;
    while (result) {


        //get other results
        result = neo4j_fetch_next(results);

        if (result) {
            neo4j_value_t node = neo4j_result_field(result, 0);
            neo4j_value_t props = neo4j_node_properties(node);
        
            Neo4jValue json(props);

            slam3d::VertexObject returnval;
            returnval.index = json["index"].as_integer();
            returnval.label = json["label"].as_string();
            returnval.robotName = json["robotName"].as_string();
            returnval.sensorName = json["sensorName"].as_string();
            returnval.typeName = json["typeName"].as_string();
            returnval.timestamp.tv_sec = json["timestamp_tv_sec"].as_integer();
            returnval.timestamp.tv_usec = json["timestamp_tv_usec"].as_integer();
            returnval.correctedPose = Eigen::Matrix4d(slam3d::Neo4jConversion::eigenMatrixFromString(json["correctedPose"].as_string()));
            returnval.measurementUuid = boost::lexical_cast<boost::uuids::uuid>(json["measurementUuid"].as_string());


            vertexobjlist.push_back(returnval);
        }
    }
    neo4j_close_results(results);
    

    // std::lock_guard<std::mutex> lock(queryMutex);
    // VertexObjectList vertexobjlist;

    // Neo4jQuery query(client);
    // query.addStatement("MATCH (n:Vertex) RETURN n AS node");
    // if (!query.sendQuery()) {
    //     std::string msg = query.getResponse().extract_string().get();
    //     logger->message(ERROR, msg);
    //     std::cout << msg << std::endl;
    //     throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    // }
    // web::json::value reply = query.getResponse().extract_json().get();
    // web::json::array& results = reply["results"][0]["data"].as_array();
    // vertexobjlist.reserve(results.size());
    // for (auto& vertex : results) {
    //     VertexObject vertexobj;
    //     vertexobj = Neo4jConversion::vertexObjectFromJson(vertex["row"][0]);
    //     vertexobjlist.push_back(vertexobj);
    // }
    return vertexobjlist;
}

float Neo4jGraph::calculateGraphDistance(IdType source_id, IdType target_id) const {
    std::lock_guard<std::mutex> lock(queryMutex);
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
    std::lock_guard<std::mutex> lock(queryMutex);
    Neo4jQuery query(client);
    // add position as extra statement (point property cannot be directly set via json props, there it is added as string)
    query.addStatement("MATCH (n:Vertex) WHERE n.index="+std::to_string(id)+" SET n.location = point({"
                            +   "x: " + std::to_string(pose.translation().x())
                            + ", y: " + std::to_string(pose.translation().y())
                            + ", z: " + std::to_string(pose.translation().z())
                            + "}) ,"
                            + " n.correctedPose = \"" + Neo4jConversion::eigenMatrixToString(pose.matrix()) + "\"");

    if (!query.sendQuery()) {
        logger->message(ERROR, query.getResponse().extract_string().get());
        throw std::runtime_error("Returned " + std::to_string(query.getResponse().status_code()) + query.getResponse().extract_string().get());
    }

}

