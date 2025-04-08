#include "Neo4jConnection.hpp"

#include <iostream>

namespace slam3d {


Neo4jConnection::Neo4jConnection(const Neo4jConnection::ServerConfig &server):lastRunSuccessful(false) {
    //neo4j://neo4j:neo4j@localhost:7687
    std::string url = "neo4j://neo4j:neo4j@"+server.host+":" + std::to_string(server.port);
    connection = neo4j_connect(url.c_str(), NULL, NEO4J_INSECURE);

    if (!connection) {
        printf("could not connect to neo4j: %s\n", url.c_str());
        exit(1);
    }
}

Neo4jConnection::~Neo4jConnection()
{
    neo4j_close(connection);
    neo4j_client_cleanup();
}

size_t Neo4jConnection::runQuery(const std::string query, std::function<void (neo4j_result_t *element)> function, neo4j_value_t params) const {
    lastRunSuccessful = false;
    if (connection) {
        // std::cout << query << std::endl;
        std::lock_guard<std::mutex> lock(queryMutex);
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
            // std::cout << query << " finshed" <<std::endl;
            lastRunSuccessful = true;
            return count;
        } else {
            // std::cout << query << " received 0 results" << std::endl;

            return 0;
        }
    } else {
        return 0;
    }
}

}  // namespace slam3d
