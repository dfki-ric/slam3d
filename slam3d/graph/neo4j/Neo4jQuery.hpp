# pragma once

#include <memory>
#include <string>

// dont define the U macro in the http client, it conflicts with the eigen U macro use _XPLATSTR() instead
#define _TURN_OFF_PLATFORM_STRING
#include <cpprest/json.h>
#include <cpprest/http_client.h>

namespace slam3d {


class Neo4jQuery {
 public:
    explicit Neo4jQuery(std::shared_ptr<web::http::client::http_client> client);

    int addStatement(const std::string& statement);

    void setStatement(const size_t &statement_index, const std::string& statement);

    void addParameterSet(const std::string& setname, const size_t &statement_index = 0);

    template <class VALUE> void addParameterToSet(const std::string& setname, const std::string& paramname, const VALUE& value, const size_t &statement_index = 0) {
        query["statements"][statement_index]["parameters"][setname][paramname] = web::json::value(value);
    }

    web::json::value* getParameterSet(const std::string& setname, size_t statement_index = 0);

    bool sendQuery();

    web::http::http_response& getResponse();

 private:
    std::shared_ptr<web::http::client::http_client> client;
    web::json::value query;
    web::http::http_response response;
    size_t next_statement_index;
};

}  // namespace slam3d
