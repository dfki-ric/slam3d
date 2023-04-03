#pragma once

#include <string>

// dont define the U macro in the http client, it conflicts with the eigen U macro use _XPLATSTR() instead
#define _TURN_OFF_PLATFORM_STRING
#include <cpprest/json.h>

#include "../../core/Types.hpp"

namespace slam3d {

class Neo4jConversion {
 public:
    static std::string eigenMatrixToString(const Eigen::MatrixXd& mat);
    static Eigen::MatrixXd eigenMatrixFromString(const std::string & string);

    static void constraintToJson(slam3d::Constraint::Ptr constraint, web::json::value* json);
    static slam3d::Constraint::Ptr jsonToConstraint(web::json::value& json);

    static slam3d::EdgeObject edgeObjectFromJson(web::json::value& json);
    static slam3d::VertexObject vertexObjectFromJson(web::json::value& json, std::map<std::string, Measurement::Ptr> &measurements);


};

}  // namespace slam3d

