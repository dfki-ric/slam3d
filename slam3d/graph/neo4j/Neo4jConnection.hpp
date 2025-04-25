# pragma once

#include <memory>
#include <string>
#include <algorithm>
#include <mutex>
#include <functional>
#include <neo4j-client.h>

namespace slam3d {

class Neo4jConnection {
    
 public:

    struct ServerConfig {
        ServerConfig(const std::string& host, const int &port, const std::string& user, const std::string& passwd) : host(host), port(port), user(user), passwd(passwd) {}
        const std::string& host;
        const int &port;
        const std::string& user;
        const std::string& passwd;
    };

    Neo4jConnection(const ServerConfig &server = ServerConfig("127.0.0.1", 7687, "neo4j", "neo4j"));
    ~Neo4jConnection();


    /**
     * @brief send a query and run fuinctio for each returned element
     * 
     * @param query 
     * @param function 
     * @return number of results
     */
    size_t runQuery(const std::string query, std::function<void (neo4j_result_t *element)> function, neo4j_value_t params = neo4j_null) const;

    bool checkSuccess() {
        return lastRunSuccessful;
    }

    // int addStatement(const std::string& statement);

    

    // void addParameterSet(const std::string& setname, const size_t &statement_index = 0);

    // template <class VALUE> void addParameterToSet(const std::string& setname, const std::string& paramname, const VALUE& value, const size_t &statement_index = 0) {
    //     query["statements"][statement_index]["parameters"][setname][paramname] = web::json::value(value);
    // }

    // template <class VALUE> void addListToSet(const std::string& setname, const std::string& paramname, const std::vector<VALUE>& values, const size_t &statement_index = 0) {
    //     std::vector<web::json::value> elements;
    //     elements.reserve(values.size());
    //     for (const auto & value : values) {
    //         elements.push_back(web::json::value(value));
    //     }
    //     query["statements"][statement_index]["parameters"][setname][paramname] = web::json::value::array(elements);
    // }


 private:
    neo4j_config *neo4jconfig;
    neo4j_connection_t *connection;
    mutable std::mutex queryMutex;
    mutable bool lastRunSuccessful;
};

}  // namespace slam3d
