# pragma once

#include <string>
#include <vector>
#include <map>

#include <boost/make_shared.hpp>

#include <slam3d/core/Types.hpp>

#include "Yaml.hpp"

/**
 * define yaml-cpp parser for the Vector3 type
 */
namespace YAML {

    /**
     * @brief slam3d::Direction and slam3d::Position
     * 
     * @tparam  
     */
    template<> struct convert<Eigen::Matrix<double, 3, 1>> {
        static bool decode(const Node& node, Eigen::Matrix<double, 3, 1>& config) {
            checkAndSet(&config.x(), node[0]);
            checkAndSet(&config.y(), node[1]);
            checkAndSet(&config.z(), node[2]);
            return true;
        }
        static Node encode(const Eigen::Matrix<double, 3, 1>& config) {
            Node node;
            node[0] = config.x();
            node[1] = config.y();
            node[2] = config.z();
            return node;
        }
    };


    template<> struct convert<slam3d::Quaternion> {
        static bool decode(const Node& node, slam3d::Quaternion& config) {
            checkAndSet(&config.x(), node[0]);
            checkAndSet(&config.y(), node[1]);
            checkAndSet(&config.z(), node[2]);
            checkAndSet(&config.w(), node[3]);
            return true;
        }
        static Node encode(const slam3d::Quaternion& config) {
            Node node;
            node[0] = config.x();
            node[1] = config.y();
            node[2] = config.z();
            node[3] = config.w();
            return node;
        }
    };

    template<> struct convert<slam3d::Transform> {
        static bool decode(const Node& node, slam3d::Transform& config) {
            // printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
            // config.translation() = node["translation"].as<slam3d::Position>();
            // slam3d::Quaternion q = node["rotation"].as<slam3d::Quaternion>();
            // q.normalize();
            // std::cout << q << std::endl;
            // config.rotate(q);
            std::vector<double> array = node.as<std::vector<double>>();
            config.matrix() = Eigen::Map<slam3d::Transform::MatrixType>(const_cast<double *>(array.data()));
            // return true;
            return true;
        }
        static Node encode(const slam3d::Transform& config) {
            Node node;
            // node["rotation"] = slam3d::Quaternion(config.linear());
            // node["translation"] = slam3d::Position(config.translation());
            std::vector<double> array;
            for (auto value : config.matrix().reshaped()) {
                array.push_back(value);
            }
            node = array;
            return node;
        }
    };

    template<> struct convert<slam3d::Covariance<6>> {
        static bool decode(const Node& node, slam3d::Covariance<6>& config) {
            std::vector<double> array = node.as<std::vector<double>>();
            config = Eigen::Map<slam3d::Covariance<6>>(const_cast<double *>(array.data()));
            return true;
        }
        static Node encode(const slam3d::Covariance<6>& config) {
            Node node;
            std::vector<double> array;
            for (auto value : config.reshaped()) {
                array.push_back(value);
            }
            node = array;
            return node;
        }
    };

    template<> struct convert<slam3d::Constraint::Ptr> {
        static bool decode(const Node& node, slam3d::Constraint::Ptr& config) {


            // for backward comp, if no type is set on loading, assume SE3
            size_t type;
            if (node["type"]) {
                type = node["type"].as<size_t>();
            }else{
                type = slam3d::SE3;
            }

            std::string sensor = node["sensor"].as<std::string>();

            switch (type) {
                case slam3d::TENTATIVE: {
                        config = boost::make_shared<slam3d::TentativeConstraint>(sensor);
                        break;
                    }
                case slam3d::SE3: {
                        slam3d::Transform tf = node["transform"].as<slam3d::Transform>();
                        slam3d::Covariance<6> cov = node["covariance_6"].as<slam3d::Covariance<6>>();
                        config = boost::make_shared<slam3d::SE3Constraint>(sensor, tf, cov);
                        break;
                    }
                case slam3d::POSE:
                    // checkAndSet(&config.translation, node["translation"]);
                    // checkAndSet(&config.rotation, node["rotation"]);
                    // checkAndSet(&config.covariance_6, node["covariance_6"]);
                    break;
                case slam3d::GRAVITY:
                    // checkAndSet(&config.mDirection, node["mDirection"]);
                    // checkAndSet(&config.mReference, node["mReference"]);
                    // checkAndSet(&config.covariance, node["covariance"]);
                    break;
                case slam3d::POSITION:
                    // checkAndSet(&config.mPosition, node["mPosition"]);
                    // checkAndSet(&config.translation, node["translation"]);
                    // checkAndSet(&config.rotation, node["rotation"]);
                    // checkAndSet(&config.covariance, node["covariance"]);
                    break;
                case slam3d::ORIENTATION:
                    // checkAndSet(&config.mOrientation, node["mOrientation"]);
                    // checkAndSet(&config.translation, node["translation"]);
                    // checkAndSet(&config.rotation, node["rotation"]);
                    // checkAndSet(&config.covariance, node["covariance"]);
                    break;
            }

            
            return true;
        }
        static Node encode(const slam3d::Constraint::Ptr& config) {
            Node node;
            node["type"] = (size_t)config->getType();
            node["sensor"] = config->getSensorName();
            // node["tv_sec"] = config->getTimestamp().tv_sec;
            // node["tv_usec"] = config->getTimestamp().tv_usec;

            switch (config->getType()) {
                case slam3d::TENTATIVE: break;
                case slam3d::SE3: {
                        slam3d::SE3Constraint& constraint = dynamic_cast<slam3d::SE3Constraint&>(*(config));
                        node["transform"] = constraint.getRelativePose();
                        node["covariance_6"] = constraint.getInformation();
                        break;
                    }
                case slam3d::POSE: {
                        slam3d::PoseConstraint& constraint = dynamic_cast<slam3d::PoseConstraint&>(*(config));
                        node["transform"] = constraint.getRelativePose();
                        node["covariance_6"] = constraint.getInformation();
                        break;
                    }
                case slam3d::GRAVITY:
                    // node["mDirection"] = config.mDirection;
                    // node["mReference"] = config.mReference;
                    // node["covariance"] = config.covariance;
                    break;
                case slam3d::POSITION:
                    // node["mPosition"] = config.mPosition;
                    // node["translation"] = config.translation;
                    // node["rotation"] = config.rotation;
                    // node["covariance"] = config.covariance;
                    break;
                case slam3d::ORIENTATION:
                    // node["mOrientation"] = config.mOrientation;
                    // node["translation"] = config.translation;
                    // node["rotation"] = config.rotation;
                    // node["covariance"] = config.covariance;
                    break;
            }
            return node;
        }
    };

    template<> struct convert<slam3d::EdgeObject> {
        static bool decode(const Node& node, slam3d::EdgeObject& config) {
            checkAndSet(&config.source, node["source"]);
            checkAndSet(&config.target, node["target"]);
            checkAndSet(&config.label, node["label"]);
            checkAndSet(&config.constraint, node["constraint"]);
            return true;
        }
        static Node encode(const slam3d::EdgeObject& config) {
            Node node;
            node["source"] = config.source;
            node["target"] = config.target;
            node["label"] = config.label;
            node["constraint"] = config.constraint;
            return node;
        }
    };


}

struct YamlVertex {
    size_t vertexIndex;
    std::string robotName;
    std::string sensorName;
    std::string typeName;
    size_t tv_sec, tv_usec, timestamp;
    std::string frame;
    std::string measurementUuid;
    std::string filename;
    slam3d::Transform correctedPose;
    slam3d::Transform sensorPose;
    bool fixed;

    std::vector<slam3d::EdgeObject> children;
};

/**
 * define yaml-cpp parser for the Config type
 */
namespace YAML {
    template<> struct convert<YamlVertex> {
        static bool decode(const Node& node, YamlVertex& config) {
            checkAndSet(&config.vertexIndex, node["vertexIndex"]);
            checkAndSet(&config.robotName, node["robotName"]);
            checkAndSet(&config.sensorName, node["sensorName"]);
            checkAndSet(&config.typeName, node["typeName"]);
            checkAndSet(&config.tv_sec, node["tv_sec"]);
            checkAndSet(&config.tv_sec, node["tv_sec"]);
            checkAndSet(&config.timestamp, node["timestamp"]);
            checkAndSet(&config.frame, node["frame"]);
            checkAndSet(&config.measurementUuid, node["measurementUuid"]);
            checkAndSet(&config.filename, node["filename"]);
            checkAndSet(&config.correctedPose, node["correctedPose"]);
            checkAndSet(&config.sensorPose, node["sensorPose"]);
            checkAndSet(&config.fixed, node["fixed"]);
            checkAndSet(&config.children, node["children"]);
            return true;
        }
        static Node encode(const YamlVertex& config) {
            Node node;
            node["vertexIndex"] = config.vertexIndex;
            node["robotName"] = config.robotName;
            node["sensorName"] = config.sensorName;
            node["typeName"] = config.typeName;
            node["tv_sec"] = config.tv_sec;
            node["tv_usec"] = config.tv_usec;
            node["timestamp"] = config.timestamp;
            node["frame"] = config.frame;
            node["measurementUuid"] = config.measurementUuid;
            node["filename"] = config.filename;
            node["correctedPose"] = config.correctedPose;
            node["sensorPose"] = config.sensorPose;
            node["fixed"] = config.fixed;
            node["children"] = config.children;
            return node;
        }
    };
}

struct YamlGraph {
    std::vector<YamlVertex> vertices;
};

namespace YAML {
    template<> struct convert<YamlGraph> {
        static bool decode(const Node& node, YamlGraph& config) {
            checkAndSet(&config.vertices, node["vertices"]);
            return true;
        }
        static Node encode(const YamlGraph& config) {
            Node node;
            node["vertices"] = config.vertices;
            return node;
        }
    };
}
