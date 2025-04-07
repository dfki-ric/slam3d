#pragma once

#include <string>

// dont define the U macro in the http client, it conflicts with the eigen U macro use _XPLATSTR() instead
#define _TURN_OFF_PLATFORM_STRING
#include <cpprest/json.h>
#include <slam3d/core/Types.hpp>

#include <neo4j-client.h>

namespace slam3d {

class MeasurementStorage;


class Neo4jValue {
 public:
    Neo4jValue(const neo4j_value_t value):value(value){}

    Neo4jValue(const neo4j_result_t *result, const size_t& index = 0){
        value = neo4j_result_field(result, 0);
    }

    int as_integer() {
        return neo4j_int_value(value);
    }

    bool as_bool() {
        return neo4j_bool_value(value);
    }

    std::string as_string() {
        std::string result;
        
        result.resize(neo4j_string_length(value)+1); // includes '\0' in copy
        neo4j_string_value(value, const_cast<char*>(result.data()), result.size());
        result.resize(result.size()-1);
        return result;
    }

    Neo4jValue as_node_properties() {
        return Neo4jValue(neo4j_node_properties(value));
    }

    Neo4jValue as_relationship_properties() {
        return Neo4jValue(neo4j_relationship_properties(value));
    }

    Neo4jValue operator[](const std::string& key) {
        return Neo4jValue(neo4j_map_kget(value, neo4j_ustring(key.data(), key.size())));
    }

    void print() {
        neo4j_fprint(value, stdout);
    }

    void printType() {
        printf("%s\n",neo4j_typestr(value._type));
    }

 private:
    neo4j_value_t value;
};



class Neo4jConversion {
 public:
    static std::string eigenMatrixToString(const Eigen::MatrixXd& mat);
    static Eigen::MatrixXd eigenMatrixFromString(const std::string & string);

    static void constraintToJson(slam3d::Constraint::Ptr constraint, web::json::value* json);
    static slam3d::Constraint::Ptr jsonToConstraint(web::json::value& json);

    static slam3d::EdgeObject edgeObjectFromJson(web::json::value& json);
    static slam3d::VertexObject vertexObjectFromJson(web::json::value& json);

    // bolt, libneo4j conversions

    static slam3d::VertexObjectList vertexObjectList(neo4j_result_stream_t *results);
    static slam3d::VertexObject vertexObject(const neo4j_result_t *result);

    static slam3d::EdgeObjectList edgeObjectList(neo4j_result_stream_t *results);
    static slam3d::EdgeObject edgeObject(const neo4j_result_t *result);

    static slam3d::Constraint::Ptr constraint(const neo4j_result_t *result);

};

}  // namespace slam3d

