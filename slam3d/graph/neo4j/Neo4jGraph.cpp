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


    //neo4j://neo4j:neo4j@localhost:7687
    std::string url = "neo4j://neo4j:neo4j@"+graphserver.host+":" + std::to_string(7687);
    connection = neo4j_connect(url.c_str(), NULL, NEO4J_INSECURE);

    if (!connection) {
        printf("could not connect to neo4j: %s\n", url.c_str());
        exit(1);
    }

}

Neo4jGraph::~Neo4jGraph()
{
    neo4j_close(connection);
    neo4j_client_cleanup();
}

bool Neo4jGraph::deleteDatabase()
{
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string request = "match (n) detach delete n";
    runQuery(request, [&](neo4j_result_t *element){});
    return true;
}

 size_t Neo4jGraph::runQuery(const std::string query, std::function<void (neo4j_result_t *element)> function, neo4j_value_t params) const {
    if (connection) {
        // std::cout << query << std::endl;
        neo4j_result_stream_t *results = neo4j_run(connection, query.c_str(), params);
        if (results) {
            neo4j_result_t *result = neo4j_fetch_next(results);
            size_t count = 0;
            while (result) {
                function(result);
                ++count;
                result = neo4j_fetch_next(results);
            }
            neo4j_close_results(results);
            return count;
        } else {
            // std::cout << query << " received 0 results" << std::endl;
            return 0;
        }
    } else {
        return 0;
    }
}

const EdgeObjectList Neo4jGraph::getEdgesFromSensor(const std::string& sensor)  const
{
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string request = "MATCH (a:Vertex)-[r]->(b:Vertex) WHERE r.sensor='"+sensor+"' AND r.inverted=false AND r.source <> r.target RETURN r";
    EdgeObjectList objectList;
    runQuery(request, [&](neo4j_result_t *element){
        objectList.push_back(Neo4jConversion::edgeObject(element));
    });
	return objectList;
}

bool Neo4jGraph::optimize(unsigned iterations)
{
    // boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
    return Graph::optimize(iterations);
}

void Neo4jGraph::addVertex(const VertexObject& v)
{
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string request = "CREATE (n:Vertex $props)";

    ParamaterSet params;
    params.addParameterSet("props");
    params.addParameterToSet("props", "label", v.label);
    params.addParameterToSet("props", "index", v.index);
    params.addParameterToSet("props", "correctedPose", Neo4jConversion::eigenMatrixToString(v.correctedPose.matrix()));
    params.addParameterToSet("props", "robotName", v.robotName);
    params.addParameterToSet("props", "sensorName", v.sensorName);
    params.addParameterToSet("props", "typeName", v.typeName);
    params.addParameterToSet("props", "timestamp_tv_sec", v.timestamp.tv_sec);
    params.addParameterToSet("props", "timestamp_tv_usec", v.timestamp.tv_usec);
    params.addParameterToSet("props", "measurementUuid", boost::lexical_cast<std::string>(v.measurementUuid));

    runQuery(request, [&](neo4j_result_t *element){}, params.get());

    // add position as extra statement (point property cannot be directly set via json props, there it is added as string)
    request = "MATCH (n:Vertex) WHERE n.index="+std::to_string(v.index)+" SET n.location = point({"
                             +   "x: " + std::to_string(v.correctedPose.translation().x())
                             + ", y: " + std::to_string(v.correctedPose.translation().y())
                             + ", z: " + std::to_string(v.correctedPose.translation().z())
                             + "})";

    runQuery(request, [&](neo4j_result_t *element){});

}

void Neo4jGraph::addEdge(const EdgeObject& e) {
    addEdge(e, true);
}

void Neo4jGraph::addEdge(const EdgeObject& e, bool addInverse) {
    std::lock_guard<std::mutex> lock(queryMutex);
    std::string constrainttypename = e.constraint->getTypeName();
    std::replace(constrainttypename.begin(), constrainttypename.end(), '(', '_');
    std::replace(constrainttypename.begin(), constrainttypename.end(), ')', '_');

    std::string request = "MATCH (a:Vertex), (b:Vertex) WHERE a.index="+std::to_string(e.source)+" AND b.index="+std::to_string(e.target) \
        + " CREATE (a)-[r:" + constrainttypename + " $props]->(b) RETURN type(r)";

    ParamaterSet params;
    params.addParameterSet("props");
    params.addParameterToSet("props", "label", e.label);
    params.addParameterToSet("props", "source", e.source);
    params.addParameterToSet("props", "target", e.target);
    params.addParameterToSet("props", "sensor", e.constraint->getSensorName());
    params.addParameterToSet("props", "timestamp_tv_sec", e.constraint->getTimestamp().tv_sec);
    params.addParameterToSet("props", "timestamp_tv_usec", e.constraint->getTimestamp().tv_usec);
    params.addBoolParameterToSet("props", "inverted", false);
    params.addParameterToSet("props", "constraint_type", e.constraint->getType());
    Neo4jConversion::constraintToParameters(e.constraint, "props", &params);

    runQuery(request, [&](neo4j_result_t *element){}, params.get());


    if (addInverse) {
        request = "MATCH (a:Vertex), (b:Vertex) WHERE a.index="+std::to_string(e.target)+" AND b.index="+std::to_string(e.source) \
                + " CREATE (a)-[r:" + constrainttypename + " $props]->(b) RETURN type(r)";

        ParamaterSet inverse_params;
        inverse_params.addParameterSet("props");
        inverse_params.addParameterToSet("props", "label", e.label);
        inverse_params.addParameterToSet("props", "source", e.target);
        inverse_params.addParameterToSet("props", "target", e.source);
        inverse_params.addParameterToSet("props", "sensor", e.constraint->getSensorName());
        inverse_params.addParameterToSet("props", "timestamp_tv_sec", e.constraint->getTimestamp().tv_sec);
        inverse_params.addParameterToSet("props", "timestamp_tv_usec", e.constraint->getTimestamp().tv_usec);
        inverse_params.addBoolParameterToSet("props", "inverted", true);
        inverse_params.addParameterToSet("props", "constraint_type", e.constraint->getType());
        Neo4jConversion::constraintToParameters(e.constraint, "props", &inverse_params);
        
        runQuery(request, [&](neo4j_result_t *element){}, inverse_params.get());
    }
}

void Neo4jGraph::removeEdge(IdType source, IdType target, const std::string& sensor) {
    std::lock_guard<std::mutex> lock(queryMutex);

    std::string request = "MATCH (a:Vertex)-[r]-(b:Vertex) WHERE a.index="+std::to_string(source)+" AND b.index="+std::to_string(target)+" AND r.sensor=\""+sensor+"\" DELETE r";
    slam3d::VertexObjectList vertexobjlist;
    runQuery(request, [&](neo4j_result_t *element){});
}


const std::set<std::string> Neo4jGraph::getVertexSensors() const {
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string request = "MATCH (a:Vertex) RETURN DISTINCT a.sensorName";
    std::set<std::string> result;
    runQuery(request, [&](neo4j_result_t *element){
        Neo4jValue val(element);
        result.insert(val.as_string());
    });
    return result;
}

const std::set<std::string> Neo4jGraph::getEdgeSensors() const {
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string request = "MATCH ()-[r]->() RETURN DISTINCT r.sensor";
    std::set<std::string> result;
    runQuery(request, [&](neo4j_result_t *element){
        Neo4jValue val(element);
        result.insert(val.as_string());
    });
    return result;
}


const VertexObjectList Neo4jGraph::getVerticesFromSensor(const std::string& sensor)  const{
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string request = "MATCH (a:Vertex) WHERE a.sensorName='"+sensor+"' RETURN a";
    slam3d::VertexObjectList vertexobjlist;
    runQuery(request, [&](neo4j_result_t *element){
        vertexobjlist.push_back(Neo4jConversion::vertexObject(element));
    });
    return vertexobjlist;
}

const VertexObjectList Neo4jGraph::getVerticesByType(const std::string& type) const {
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string request = "MATCH (a:Vertex) WHERE a.typeName='"+type+"' RETURN a";
    slam3d::VertexObjectList vertexobjlist;
    runQuery(request, [&](neo4j_result_t *element){
        vertexobjlist.push_back( Neo4jConversion::vertexObject(element));
    });
    return vertexobjlist;
}

const VertexObjectList Neo4jGraph::getNearbyVertices(const Transform &location, float radius, const std::string& sensortype) const {
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string request;
    if (sensortype == "") {
        request = "MATCH (a:Vertex) WHERE point.distance(point({x:"+std::to_string(location.translation().x())+", y:"+std::to_string(location.translation().y())+", z:"+std::to_string(location.translation().z())+"}), a.location) < "+std::to_string(radius)+" RETURN a";
    } else {
        request = "MATCH (a:Vertex) WHERE point.distance(point({x:"+std::to_string(location.translation().x())+", y:"+std::to_string(location.translation().y())+", z:"+std::to_string(location.translation().z())+"}), a.location) < "+std::to_string(radius)+" AND a.typeName = \""+sensortype+"\" RETURN a";
    }
    slam3d::VertexObjectList vertexobjlist;
    runQuery(request, [&](neo4j_result_t *element){
        vertexobjlist.push_back( Neo4jConversion::vertexObject(element));
    });
    return vertexobjlist;
}

const VertexObject Neo4jGraph::getVertex(IdType id)  const {
    std::string request = "MATCH (n:Vertex) WHERE n.index="+std::to_string(id)+" RETURN n";
    slam3d::VertexObject vertexobj;
    runQuery(request, [&](neo4j_result_t *element){
        vertexobj = Neo4jConversion::vertexObject(element);
    });
    return vertexobj;
}

const VertexObject Neo4jGraph::getVertex(boost::uuids::uuid id) const {
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string uuid = boost::lexical_cast<std::string>(id);
    std::string request = "MATCH (n:Vertex) WHERE n.measurementUuid = "+uuid+" RETURN n AS node";
    slam3d::VertexObject vertexobj;
    runQuery(request, [&](neo4j_result_t *element){
        vertexobj = Neo4jConversion::vertexObject(element);
    });
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
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string request = "MATCH (a:Vertex)-[r]->(b:Vertex) WHERE a.index="+std::to_string(source)+" AND b.index="+std::to_string(target)+" AND r.sensor='"+sensor+"' RETURN r";
    EdgeObject object;
    size_t found = runQuery(request, [&](neo4j_result_t *element) {
        object = Neo4jConversion::edgeObject(element);
    });
    if (found == 0) {
        throw InvalidEdge(source, target);
    }
    return object;
}

const EdgeObjectList Neo4jGraph::getOutEdges(IdType source) const {

    std::string request = "MATCH (a:Vertex)-[r]->() WHERE a.index="+std::to_string(source)+" RETURN r";
    
    EdgeObjectList objectList;
    runQuery(request, [&](neo4j_result_t *element){
        objectList.push_back(Neo4jConversion::edgeObject(element));
    });
    return objectList;
}

const EdgeObjectList Neo4jGraph::getEdges(const VertexObjectList& vertices) const {
    // sort ids into set
	std::set<int> v_ids;

	for(VertexObjectList::const_iterator v = vertices.begin(); v != vertices.end(); v++) {
		v_ids.insert(v->index);
	}
    //generate query list:
    std::string list = "[";
    for (const auto& id: v_ids) {
        list += std::to_string(id) + ",";
    }
    //replace last , with ]
    list.back() = ']';

    std::string request = "MATCH (a)-[r]->(b) where a.index IN "+list+" AND b.index IN "+list+" RETURN r";
    std::cout << request << std::endl;
    neo4j_result_stream_t *results = neo4j_run(connection, request.c_str(), neo4j_null);
    EdgeObjectList objectList = Neo4jConversion::edgeObjectList(results);
    neo4j_close_results(results);
	
	return objectList;
}

void Neo4jGraph::writeGraphToFile(const std::string& name)
{
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
}

const VertexObjectList Neo4jGraph::getVerticesInRange(IdType source_id, unsigned range) const
{
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string request = "match (v1:Vertex)--{1,"+std::to_string(range)+"}(v2:Vertex) where v1.index="+std::to_string(source_id)+" return v2 as node";
    neo4j_result_stream_t *results = neo4j_run(connection, request.c_str(), neo4j_null);
    slam3d::VertexObjectList vertexobjlist = Neo4jConversion::vertexObjectList(results);
    neo4j_close_results(results);
    return vertexobjlist;
}

const VertexObjectList Neo4jGraph::getAllVertices() const {
    // std::lock_guard<std::mutex> lock(queryMutex);
    std::string request = "MATCH (n:Vertex) RETURN n AS node";
    neo4j_result_stream_t *results = neo4j_run(connection, request.c_str(), neo4j_null);
    slam3d::VertexObjectList vertexobjlist = Neo4jConversion::vertexObjectList(results);
    neo4j_close_results(results);
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

