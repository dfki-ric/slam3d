#include "Neo4jConversion.hpp"

#include <slam3d/core/MeasurementStorage.hpp>
#include <boost/lexical_cast.hpp>

namespace slam3d {

std::string Neo4jConversion::eigenMatrixToString(const Eigen::MatrixXd& mat) {
    std::stringstream ss;
    // the '\n' id not needed but makes the value more readable in the neo4j browser
    Eigen::IOFormat jsonfmt(Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");
    ss << mat.format(jsonfmt);
    return ss.str();
}

Eigen::MatrixXd Neo4jConversion::eigenMatrixFromString(const std::string & string) {
    web::json::value val = web::json::value::parse(string);
    Eigen::MatrixXd mat(val.size(), val[0].size());
    for (size_t y = 0; y < val[0].size(); ++y) {
        for (size_t x = 0; x < val.size(); ++x) {
            mat(x, y) = val[x][y].as_double();
        }
    }
    return mat;
}

slam3d::EdgeObject Neo4jConversion::edgeObjectFromJson(web::json::value& json) {
    slam3d::EdgeObject returnval;

    if (json["inverted"].as_bool()) {
        returnval.source = json["target"].as_integer();
        returnval.target = json["source"].as_integer();
    } else {
        returnval.source = json["source"].as_integer();
        returnval.target = json["target"].as_integer();
    }
    returnval.constraint = Neo4jConversion::jsonToConstraint(json);
    return returnval;
}

slam3d::VertexObject Neo4jConversion::vertexObjectFromJson(web::json::value& json) {
    slam3d::VertexObject returnval;
    returnval.index = json["index"].as_integer();
    returnval.label = json["label"].as_string();
    returnval.mRobotName = json["mRobotName"].as_string();
    returnval.mSensorName = json["mSensorName"].as_string();
    returnval.mTypeName = json["mTypeName"].as_string();
    returnval.mStamp.tv_sec = json["timestamp_tv_sec"].as_integer();
    returnval.mStamp.tv_usec = json["timestamp_tv_usec"].as_integer();
    returnval.mTypeName = json["mTypeName"].as_string();

    returnval.corrected_pose = Eigen::Matrix4d(Neo4jConversion::eigenMatrixFromString(json["corrected_pose"].as_string()));
    returnval.measurement_uuid = boost::lexical_cast<boost::uuids::uuid>(json["measurement"].as_string());
    return returnval;
}

void Neo4jConversion::constraintToJson(slam3d::Constraint::Ptr constraint, web::json::value* json) {
    web::json::value& data = *json;

    switch (constraint->getType()) {
        case slam3d::TENTATIVE : break;
        case slam3d::SE3 : {
            slam3d::SE3Constraint* se3 = dynamic_cast<slam3d::SE3Constraint*>(constraint.get());
            data["mRelativePose"] = web::json::value(eigenMatrixToString(se3->getRelativePose().matrix()));
            data["mInformation"] = web::json::value(eigenMatrixToString(se3->getInformation().matrix()));
            break;
        }
        case slam3d::GRAVITY : {
            slam3d::GravityConstraint* grav = dynamic_cast<slam3d::GravityConstraint*>(constraint.get());
            data["mDirection"] = web::json::value(eigenMatrixToString(grav->getDirection().matrix()));
            data["mReference"] = web::json::value(eigenMatrixToString(grav->getReference().matrix()));
            data["mCovariance"] = web::json::value(eigenMatrixToString(grav->getCovariance().matrix()));
            break;
        }
        case slam3d::POSITION : {
            slam3d::PositionConstraint* grav = dynamic_cast<slam3d::PositionConstraint*>(constraint.get());
            data["mPosition"] = web::json::value(eigenMatrixToString(grav->getPosition().matrix()));
            data["mSensorPose"] = web::json::value(eigenMatrixToString(grav->getSensorPose().matrix()));
            data["mCovariance"] = web::json::value(eigenMatrixToString(grav->getCovariance().matrix()));
            break;
        }
        case slam3d::ORIENTATION : {
            slam3d::OrientationConstraint* grav = dynamic_cast<slam3d::OrientationConstraint*>(constraint.get());
            data["mOrientation"] = web::json::value(eigenMatrixToString(grav->getOrientation().matrix()));
            data["mSensorPose"] = web::json::value(eigenMatrixToString(grav->getSensorPose().matrix()));
            data["mCovariance"] = web::json::value(eigenMatrixToString(grav->getCovariance().matrix()));
            break;
        }
    }
}

slam3d::Constraint::Ptr Neo4jConversion::jsonToConstraint(web::json::value& json) {
    switch (json["constraint_type"].as_integer()) {
        case slam3d::TENTATIVE : return slam3d::Constraint::Ptr(new slam3d::TentativeConstraint(json["sensor"].as_string()));
        case slam3d::SE3 : {
            slam3d::Transform t = slam3d::Transform(Eigen::Matrix4d(eigenMatrixFromString(json["mRelativePose"].as_string())));
            slam3d::Covariance<6> i = slam3d::Covariance<6>(eigenMatrixFromString(json["mInformation"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::SE3Constraint(json["sensor"].as_string(), t, i));
        }
        case slam3d::GRAVITY : {
            slam3d::Direction d = slam3d::Direction(eigenMatrixFromString(json["mDirection"].as_string()));
            slam3d::Direction r = slam3d::Direction(eigenMatrixFromString(json["mReference"].as_string()));
            slam3d::Covariance<2> c = slam3d::Covariance<2>(eigenMatrixFromString(json["mCovariance"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::GravityConstraint(json["sensor"].as_string(), d, r, c));
        }
        case slam3d::POSITION : {
            slam3d::Position p = slam3d::Position(eigenMatrixFromString(json["mPosition"].as_string()));
            slam3d::Transform t = slam3d::Transform(Eigen::Matrix4d(eigenMatrixFromString(json["mSensorPose"].as_string())));
            slam3d::Covariance<3> c = slam3d::Covariance<3>(eigenMatrixFromString(json["mCovariance"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::PositionConstraint(json["sensor"].as_string(), p, c, t));
        }
        case slam3d::ORIENTATION : {
            slam3d::Quaternion q = slam3d::Quaternion(Eigen::Matrix3d(eigenMatrixFromString(json["mOrientation"].as_string())));
            slam3d::Transform t = slam3d::Transform(Eigen::Matrix4d(eigenMatrixFromString(json["mSensorPose"].as_string())));
            slam3d::Covariance<3> c = slam3d::Covariance<3>(eigenMatrixFromString(json["mCovariance"].as_string()));
            return slam3d::Constraint::Ptr(new slam3d::OrientationConstraint(json["sensor"].as_string(), q, c, t));
        }
    }
    // should never be here
    return slam3d::Constraint::Ptr();
}



}  // namespace slam3d

