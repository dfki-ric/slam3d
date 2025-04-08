#pragma once

#include <string>
// dont define the U macro in the http client, it conflicts with the eigen U macro use _XPLATSTR() instead
// #define _TURN_OFF_PLATFORM_STRING
// #include <cpprest/json.h>
#include <slam3d/core/Types.hpp>

#include <neo4j-client.h>

#include "Neo4jParamaterSet.hpp"

namespace slam3d {

class MeasurementStorage;


class Neo4jConversion {
 public:
    static std::string eigenMatrixToString(const Eigen::MatrixXd& mat);
    static Eigen::MatrixXd eigenMatrixFromString(const std::string & string);

    // static void constraintToJson(slam3d::Constraint::Ptr constraint, web::json::value* json);
    // static slam3d::Constraint::Ptr jsonToConstraint(web::json::value& json);

    // static slam3d::EdgeObject edgeObjectFromJson(web::json::value& json);
    // static slam3d::VertexObject vertexObjectFromJson(web::json::value& json);

    // bolt, libneo4j conversions

    // static slam3d::VertexObjectList vertexObjectList(neo4j_result_stream_t *results);
    static slam3d::VertexObject vertexObject(const neo4j_result_t *result);

    // static slam3d::EdgeObjectList edgeObjectList(neo4j_result_stream_t *results);
    static slam3d::EdgeObject edgeObject(const neo4j_result_t *result);

    static slam3d::Constraint::Ptr constraint(const neo4j_result_t *result);
    static bool constraint(const slam3d::Constraint::Ptr, neo4j_result_t *result);

    static void constraintToParameters(slam3d::Constraint::Ptr constraint, const std::string& setname, ParamaterSet* set);

    

};

}  // namespace slam3d

