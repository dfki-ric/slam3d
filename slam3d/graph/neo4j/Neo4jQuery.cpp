#include "Neo4jQuery.hpp"

namespace slam3d {

Neo4jQuery::Neo4jQuery(std::shared_ptr<web::http::client::http_client> client) : client(client), next_statement_index(0) {
    // query = web::json::value::object(true); sorted
    query["statements"] = web::json::value::array(0);
    // query["statements"][0]["parameters"] = web::json::value();
}

int Neo4jQuery::addStatement(const std::string& statement) {
    query["statements"][next_statement_index]["statement"] = web::json::value(statement);
    ++next_statement_index;
    return next_statement_index-1;
}

void Neo4jQuery::setStatement(const size_t &statement_index, const std::string& statement) {
        query["statements"][statement_index]["statement"] = web::json::value(statement);
}

void Neo4jQuery::addParameterSet(const std::string& setname, const size_t &statement_index) {
    query["statements"][statement_index]["parameters"][setname] = web::json::value();
}

web::json::value* Neo4jQuery::getParameterSet(const std::string& setname, size_t statement_index) {
    if (query["statements"][statement_index]["parameters"][setname].is_null()) {
        addParameterSet(setname);
    }
    return &(query["statements"][statement_index]["parameters"][setname]);
}

bool Neo4jQuery::sendQuery() {
    // response = web::http::http_response();
    // std::cout << query.serialize() << std::endl;
    response = client->request(web::http::methods::POST, "/db/neo4j/tx/commit", query.serialize(), "application/json;charset=utf-8").get();
    if (response.status_code() != 200) {
        std::cout << "ERROR " << response.to_string() << std::endl;
        return false;
    }
    return true;
}

web::http::http_response& Neo4jQuery::getResponse() {
    return response;
}


void Neo4jQuery::printQuery() {
    std::cout << query << std::endl;
}

}  // namespace slam3d
