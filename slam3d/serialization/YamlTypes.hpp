# pragma once

#include <string>
#include <vector>

#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>

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
            std::vector<double> array = node.as<std::vector<double>>();
            config.matrix() = Eigen::Map<slam3d::Transform::MatrixType>(const_cast<double *>(array.data()));
            return true;
        }
        static Node encode(const slam3d::Transform& config) {
            Node node;
            std::vector<double> array;
            for (auto value : config.matrix().reshaped()) {
                array.push_back(value);
            }
            node = array;
            return node;
        }
    };

    template<> struct convert<slam3d::Covariance<2>> {
        static bool decode(const Node& node, slam3d::Covariance<2>& config) {
            std::vector<double> array = node.as<std::vector<double>>();
            config = Eigen::Map<slam3d::Covariance<2>>(const_cast<double *>(array.data()));
            return true;
        }
        static Node encode(const slam3d::Covariance<2>& config) {
            Node node;
            std::vector<double> array;
            for (auto value : config.reshaped()) {
                array.push_back(value);
            }
            node = array;
            return node;
        }
    };

    template<> struct convert<slam3d::Covariance<3>> {
        static bool decode(const Node& node, slam3d::Covariance<3>& config) {
            std::vector<double> array = node.as<std::vector<double>>();
            config = Eigen::Map<slam3d::Covariance<3>>(const_cast<double *>(array.data()));
            return true;
        }
        static Node encode(const slam3d::Covariance<3>& config) {
            Node node;
            std::vector<double> array;
            for (auto value : config.reshaped()) {
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

    template<> struct convert<slam3d::Constraint::Ptr>
	{
        static bool decode(const Node& node, slam3d::Constraint::Ptr& config)
		{
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
                        // config = boost::make_shared<slam3d::TentativeConstraint>(sensor);
                        break;
                    }
                case slam3d::SE3: {
                        slam3d::Transform tf = node["transform"].as<slam3d::Transform>();
                        slam3d::Covariance<6> cov = node["covariance_6"].as<slam3d::Covariance<6>>();
                        config = boost::make_shared<slam3d::SE3Constraint>(sensor, tf, cov);
                        break;
                    }
                case slam3d::POSE: {
                        slam3d::Transform tf = node["transform"].as<slam3d::Transform>();
                        slam3d::Covariance<6> cov = node["covariance_6"].as<slam3d::Covariance<6>>();
                        config = boost::make_shared<slam3d::PoseConstraint>(sensor, tf, cov);
                        break;
                    }
                case slam3d::GRAVITY: {
                        slam3d::Direction d1 = node["mDirection"].as<slam3d::Direction>();
                        slam3d::Direction d2 = node["mReference"].as<slam3d::Direction>();
                        slam3d::Covariance<2> cov = node["covariance_2"].as<slam3d::Covariance<2>>();
                        config = boost::make_shared<slam3d::GravityConstraint>(sensor, d1, d2, cov);
                        break;
                    }
                case slam3d::POSITION: {
                        slam3d::Position p = node["mPosition"].as<slam3d::Direction>();
                        slam3d::Covariance<3> cov = node["covariance_3"].as<slam3d::Covariance<3>>();
                        slam3d::Transform sp = node["mSensorPose"].as<slam3d::Transform>();
                        config = boost::make_shared<slam3d::PositionConstraint>(sensor, p, cov, sp);
                        break;
                    }
                case slam3d::ORIENTATION: {
                        slam3d::Quaternion p = node["mOrientation"].as<slam3d::Quaternion>();
                        slam3d::Covariance<3> cov = node["covariance_3"].as<slam3d::Covariance<3>>();
                        slam3d::Transform sp = node["mSensorPose"].as<slam3d::Transform>();
                        config = boost::make_shared<slam3d::OrientationConstraint>(sensor, p, cov, sp);
                        break;
                    }
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
                case slam3d::GRAVITY: {
                        slam3d::GravityConstraint& constraint = dynamic_cast<slam3d::GravityConstraint&>(*(config));
                        node["mDirection"] = constraint.getDirection();
                        node["mReference"] = constraint.getReference();
                        node["covariance_2"] = constraint.getCovariance();
                        break;
                    }
                case slam3d::POSITION: {
                        slam3d::PositionConstraint& constraint = dynamic_cast<slam3d::PositionConstraint&>(*(config));
                        node["mPosition"] = constraint.getPosition();
                        node["covariance_3"] = constraint.getCovariance();
                        node["mSensorPose"] = constraint.getSensorPose();
                        break;
                    }
                case slam3d::ORIENTATION: {
                        slam3d::OrientationConstraint& constraint = dynamic_cast<slam3d::OrientationConstraint&>(*(config));
                        node["mOrientation"] = constraint.getOrientation();
                        node["covariance_3"] = constraint.getCovariance();
                        node["mSensorPose"] = constraint.getSensorPose();
                        break;
                    }
            }
            if (!node) {
                throw std::runtime_error("couls not load constraint of sensor: " + config->getSensorName());
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

	template<> struct convert<boost::uuids::uuid>
	{
		static bool decode(const Node& node, boost::uuids::uuid& id)
		{
			if (node)
			{
				id = node.as<boost::uuids::uuid>();
				return true;
			}
			return false;
		}
		static Node encode(const boost::uuids::uuid& id)
		{
            Node node;
            node = id;
            return node;
        }
	};

	template<> struct convert<timeval>
	{
		static bool decode(const Node& node, timeval& time)
		{
			checkAndSet(&time.tv_sec, node["tv_sec"]);
			checkAndSet(&time.tv_usec, node["tv_usec"]);
			return true;
		}
		static Node encode(const timeval& time)
		{
            Node node;
            node["tv_sec"] = time.tv_sec;
			node["tv_usec"] = time.tv_usec;
            return node;
        }
	};

    template<> struct convert<slam3d::VertexObject>
	{
        static bool decode(const Node& node, slam3d::VertexObject& config)
		{
            checkAndSet(&config.index, node["index"]);
            checkAndSet(&config.timestamp, node["timestamp"]);
            checkAndSet(&config.label, node["label"]);
            checkAndSet(&config.robotName, node["robotName"]);
            checkAndSet(&config.sensorName, node["sensorName"]);
            checkAndSet(&config.typeName, node["typeName"]);
            checkAndSet(&config.correctedPose, node["correctedPose"]);
            checkAndSet(&config.fixed, node["fixed"]);
            checkAndSet(&config.measurementUuid,node["measurementUuid"]);

            return true;
        }
        static Node encode(const slam3d::VertexObject& config)
		{
            Node node;
            node["index"] = config.index;
            node["timestamp"] = config.timestamp;
            node["label"] = config.label;
            node["robotName"] = config.robotName;
            node["sensorName"] = config.sensorName;
            node["typeName"] = config.typeName;
            node["measurementUuid"] = config.measurementUuid;
            node["correctedPose"] = config.correctedPose;
            node["fixed"] = config.fixed;
            return node;
        }
    };
}

struct YamlGraph
{
    slam3d::VertexObjectList vertices;
	slam3d::EdgeObjectList edges;
};

namespace YAML
{
	template<> struct convert<YamlGraph>
	{
		static bool decode(const Node& node, YamlGraph& config)
		{
			checkAndSet(&config.vertices, node["vertices"]);
			checkAndSet(&config.edges, node["edges"]);
			return true;
		}
		static Node encode(const YamlGraph& config)
		{
			Node node;
			node["vertices"] = config.vertices;
			node["edges"] = config.edges;
			return node;
		}
	};
}
