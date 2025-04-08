#include "Neo4jConversion.hpp"

#include <slam3d/core/MeasurementStorage.hpp>
#include <boost/lexical_cast.hpp>
#include <jsoncpp/json/json.h>

#include "Neo4jGraph.hpp"
#include "Neo4jValue.hpp"

namespace slam3d {

std::string Neo4jConversion::eigenMatrixToString(const Eigen::MatrixXd& mat) {
    std::stringstream ss;
    // the '\n' id not needed but makes the value more readable in the neo4j browser
    Eigen::IOFormat jsonfmt(Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");
    ss << mat.format(jsonfmt);
    return ss.str();
}

Eigen::MatrixXd Neo4jConversion::eigenMatrixFromString(const std::string & string) {

    Json::Reader reader;
    Json::Value val;
    // web::json::value val = web::json::value::parse(string);
    
    if (!reader.parse( string, val )) {
        printf("error readign matrix \n");
    }

    Eigen::MatrixXd mat(val.size(), val[0].size());
    for (int y = 0; y < val[0].size(); ++y) {
        for (int  x = 0; x < val.size(); ++x) {
            mat(x, y) = val[x][y].asDouble();
        }
    }
    return mat;
}

slam3d::VertexObject Neo4jConversion::vertexObject(const neo4j_result_t *result) {
    slam3d::VertexObject returnval;
    Neo4jValue bolt(result);
    Neo4jValue properties = bolt.as_node_properties();

    returnval.index = properties["index"].as_integer();
    returnval.label = properties["label"].as_string();
    returnval.robotName = properties["robotName"].as_string();
    returnval.sensorName = properties["sensorName"].as_string();
    returnval.typeName = properties["typeName"].as_string();
    returnval.timestamp.tv_sec = properties["timestamp_tv_sec"].as_integer();
    returnval.timestamp.tv_usec = properties["timestamp_tv_usec"].as_integer();
    returnval.correctedPose = Eigen::Matrix4d(slam3d::Neo4jConversion::eigenMatrixFromString(properties["correctedPose"].as_string()));
    returnval.measurementUuid = boost::lexical_cast<boost::uuids::uuid>(properties["measurementUuid"].as_string());
    return returnval;
}


slam3d::EdgeObject Neo4jConversion::edgeObject(const neo4j_result_t *result) {
    slam3d::EdgeObject returnval;
    Neo4jValue bolt(result);
    Neo4jValue properties = bolt.as_relationship_properties();

    if (properties["inverted"].as_bool()) {
        returnval.source = properties["target"].as_integer();
        returnval.target = properties["source"].as_integer();
    } else {
        returnval.source = properties["source"].as_integer();
        returnval.target = properties["target"].as_integer();
    }
    //
    returnval.constraint = Neo4jConversion::constraint(result);
    return returnval;
}

slam3d::Constraint::Ptr Neo4jConversion::constraint(const neo4j_result_t *result) {
    Neo4jValue bolt(result);
    Neo4jValue properties = bolt.as_relationship_properties();
    switch (properties["constraint_type"].as_integer()) {
        case slam3d::TENTATIVE : return slam3d::Constraint::Ptr(new slam3d::TentativeConstraint(properties["sensor"].as_string()));
        case slam3d::SE3 : {
            slam3d::Transform t = slam3d::Transform(Eigen::Matrix4d(eigenMatrixFromString(properties["mRelativePose"].as_string())));
            slam3d::Covariance<6> i = slam3d::Covariance<6>(eigenMatrixFromString(properties["mInformation"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::SE3Constraint(properties["sensor"].as_string(), t, i));
        }
        case slam3d::GRAVITY : {
            slam3d::Direction d = slam3d::Direction(eigenMatrixFromString(properties["mDirection"].as_string()));
            slam3d::Direction r = slam3d::Direction(eigenMatrixFromString(properties["mReference"].as_string()));
            slam3d::Covariance<2> c = slam3d::Covariance<2>(eigenMatrixFromString(properties["mCovariance"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::GravityConstraint(properties["sensor"].as_string(), d, r, c));
        }
        case slam3d::POSITION : {
            slam3d::Position p = slam3d::Position(eigenMatrixFromString(properties["mPosition"].as_string()));
            slam3d::Transform t = slam3d::Transform(Eigen::Matrix4d(eigenMatrixFromString(properties["mSensorPose"].as_string())));
            slam3d::Covariance<3> c = slam3d::Covariance<3>(eigenMatrixFromString(properties["mCovariance"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::PositionConstraint(properties["sensor"].as_string(), p, c, t));
        }
        case slam3d::ORIENTATION : {
            slam3d::Quaternion q = slam3d::Quaternion(Eigen::Matrix3d(eigenMatrixFromString(properties["mOrientation"].as_string())));
            slam3d::Transform t = slam3d::Transform(Eigen::Matrix4d(eigenMatrixFromString(properties["mSensorPose"].as_string())));
            slam3d::Covariance<3> c = slam3d::Covariance<3>(eigenMatrixFromString(properties["mCovariance"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::OrientationConstraint(properties["sensor"].as_string(), q, c, t));
        }
    }
    // should never be here
    return slam3d::Constraint::Ptr();
}

void  Neo4jConversion::constraintToParameters(slam3d::Constraint::Ptr constraint, const std::string& setname, ParamaterSet* set) {
    ParamaterSet& params = *set;

    switch (constraint->getType()) {
        case slam3d::TENTATIVE : break;
        case slam3d::SE3 : {
            slam3d::SE3Constraint* se3 = dynamic_cast<slam3d::SE3Constraint*>(constraint.get());
            params.addParameterToSet(setname, "mRelativePose", eigenMatrixToString(se3->getRelativePose().matrix()));
            params.addParameterToSet(setname, "mInformation", eigenMatrixToString(se3->getInformation().matrix()));
            break;
        }
        case slam3d::GRAVITY : {
            slam3d::GravityConstraint* grav = dynamic_cast<slam3d::GravityConstraint*>(constraint.get());
            params.addParameterToSet(setname, "mDirection", eigenMatrixToString(grav->getDirection().matrix()));
            params.addParameterToSet(setname, "mReference", eigenMatrixToString(grav->getReference().matrix()));
            params.addParameterToSet(setname, "mCovariance", eigenMatrixToString(grav->getCovariance().matrix()));
            break;
        }
        case slam3d::POSITION : {
            slam3d::PositionConstraint* grav = dynamic_cast<slam3d::PositionConstraint*>(constraint.get());
            params.addParameterToSet(setname, "mPosition", eigenMatrixToString(grav->getPosition().matrix()));
            params.addParameterToSet(setname, "mSensorPose", eigenMatrixToString(grav->getSensorPose().matrix()));
            params.addParameterToSet(setname, "mCovariance", eigenMatrixToString(grav->getCovariance().matrix()));
            break;
        }
        case slam3d::ORIENTATION : {
            slam3d::OrientationConstraint* grav = dynamic_cast<slam3d::OrientationConstraint*>(constraint.get());
            params.addParameterToSet(setname, "mOrientation", eigenMatrixToString(grav->getOrientation().matrix()));
            params.addParameterToSet(setname, "mSensorPose", eigenMatrixToString(grav->getSensorPose().matrix()));
            params.addParameterToSet(setname, "mCovariance", eigenMatrixToString(grav->getCovariance().matrix()));
            break;
        }
    }

}


}  // namespace slam3d

