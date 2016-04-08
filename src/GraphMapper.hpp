#ifndef SLAM_GRAPHMAPPER_HPP
#define SLAM_GRAPHMAPPER_HPP

/**
 * @mainpage A generic frontend for 3D Simultaneous Localization and Mapping
 * 
 * @section sec_motiv Motivation
 * 
 * This library provides a frontend for a graph-based SLAM in three dimensional space.
 * It does not provide a graph-optimization-backend itself (often referred to as SLAM).
 * Instead different backends can be used by implementing the Solver-Interface.
 * 
 * @section sec_start Getting started
 * 
 * The central component of this library is the GraphMapper class.
 * The documentation is best read by starting from there.
 * This class is extended by registering Sensor modules, an Odometry and a Solver.
 * 
 * @section sec_example Programming example
 * 
 * Start by creating the mapper itself and registering the required modules.
 @code
#include <slam3d/GraphMapper.hpp> 
#include <slam3d/G2oSolver.hpp>

using namespace slam3d;
Clock* clock = new Clock();
Logger* logger = new Logger(*c);
GraphMapper* mapper = new GraphMapper(logger);

Sensor* laser = new PointCloudSensor("laser", logger, Transform::Identity());
mapper->registerSensor(laser);

G2oSolver* g2o = new G2oSolver(logger);
mapper->setSolver(g2o);
 @endcode
 * Within the callback of your sensor data, add the new measurements to the mapper.
 @code
Measurement m* = new PointCloudMeasurement(cloud, "my_robot", laser->getName(), laser->getSensorPose());
if(!mapper->addReading(m))
{
  delete m;
}
 @endcode
 */

#include "Odometry.hpp"
#include "Sensor.hpp"
#include "Solver.hpp"

#include <map>

namespace slam3d
{
	/**
	 * @class InvalidEdge
	 * @brief Exception thrown when a specified edge does not exist.
	 */
	class InvalidEdge : public std::exception
	{
	public:
		InvalidEdge(IdType s, IdType t)
		: source(s), target(t) {}
		
		virtual const char* what() const throw()
		{
			std::ostringstream msg;
			msg << "No edge between " << source << " and " << target << "!";
			return msg.str().c_str();
		}
		
		IdType source;
		IdType target;
	};

	/**
	 * @class DuplicateEdge
	 * @brief Exception thrown when an added edge already exists.
	 */
	class DuplicateEdge : public std::exception
	{
	public:
		DuplicateEdge(IdType s, IdType t, const std::string& name)
		 : source(s), target(t), sensor(name) {}
		~DuplicateEdge() throw() {}
		
		virtual const char* what() const throw()
		{
			std::ostringstream msg;
			msg << "Edge between " << source << " and " << target << " from sensor " << sensor << "already exists!";
			return msg.str().c_str();
		}
		
		IdType source;
		IdType target;
		std::string sensor;
	};

	/**
	 * @class DuplicateMeasurement
	 * @brief Exception thrown when an added measurement already exists.
	 */
	class DuplicateMeasurement: public std::exception
	{
	public:
		virtual const char* what() const throw()
		{
			std::ostringstream msg;
			msg << "Measurement already in graph!";
			return msg.str().c_str();
		}
	};
	/**
	 * @class GraphMapper
	 * @brief Holds measurements from different sensors in a graph.
	 * @details The GraphMapper is the central structure that provides the
	 * frontend for a graph-based SLAM approach. A registered Sensor
	 * will provide a specific Measurement type to the internal graph.
	 * For each added measurement a new vertex is created in the graph
	 * that holds a pointer to the measurement together with the measurement's
	 * pose in the map coordinate frame. This data is stored in a VertexObject.
	 * 
	 * Spatial relations between measurements are represented as edges in the
	 * graph. A registered Odometry will provide spatial constraints between any
	 * kind of consecutive measurements. Each sensor can create constraints
	 * between its own measurements by applying some kind of matching algoithm.
	 * This kind of 6 DoF spatial relation is stored as transform and covariance
	 * within an EdgeObject.
	 * 
	 * The global optimization is provided by a Solver that takes the internal
	 * nodes and edges (without the measurements) and solves the SLAM problem by
	 * applying a graph optimization algorithm. This will usually change the
	 * poses of all nodes in the map coordinate frame.
	 */
	class GraphMapper
	{
	public:
		GraphMapper(Logger* log);
		virtual ~GraphMapper();

		/**
		 * @brief Sets a specific Solver to be used as SLAM backend.
		 * @details The mapper can be used without a backend,
		 * but mapping results might be inconsistant.
		 * @param solver backend to be used for optimization
		 */
		void setSolver(Solver* solver);

		/**
		 * @brief Sets an odometry module to provide relative poses 
		 * @details Depending on the matching abilities of the
		 * used sensors (e.g. a 360° laser-scanner), the mapping might work
		 * correctly without an odometry module.
		 * @param odom odometry module
		 * @param add_edges whether to create odometry edges in the graph
		 */
		void setOdometry(Odometry* odom, bool add_edges = false);

		/**
		 * @brief Register a sensor, so its data can be added to the graph.
		 * @details Multiple sensors can be used, but in this case an odometry module
		 * is required for the mapping to work correctly. Matching is currently
		 * done only between measurements of the same sensor.
		 * @param s sensor to be registered for mapping
		 */
		void registerSensor(Sensor* s);

		/**
		 * @brief Add a new measurement to the graph.
		 * @details The sensor specified in the measurement has to be registered
		 * with the mapper before. If the change in robot pose since the last
		 * added scan is smaller then min-translation or min-rotation, the
		 * measurement will not be added. Use GraphMapper::setMinPoseDistance to
		 * adjust this distance.
		 * @param m pointer to a new measurement
		 * @return true if the measurement was added
		 */
		virtual bool addReading(Measurement::Ptr m) = 0;
		
		/**
		 * @brief Add a new measurement from another robot.
		 * @details The new measurement is added to the graph and directly
		 * linked to the measurement with the given uuid. This enforces that
		 * the graph stays connected even when external measurement cannot be
		 * linked to local ones.
		 * @param measurement pointer to a new measurement
		 * @param source_uuid uuid of another measurement
		 * @param tf transform between measurement and source
		 * @param cov covariance of that transform
		 * @param sensor name of sensor that created the constraint (not the measurement!)
		 */
		virtual void addExternalReading(Measurement::Ptr measurement,
		                                boost::uuids::uuid source_uuid,
		                                const Transform& tf,
		                                const Covariance& cov,
										const std::string& sensor) = 0;

		/**
		 * @brief Add a constraint from another robot between two measurements.
		 * @param source uuid of a measurement
		 * @param target uuid of a measurement
		 * @param relative_pose transform from source to target
		 * @param covariance covarinave of that transform
		 * @param sensor name of sensor that created the constraint
		 */
		virtual void addExternalConstraint(boost::uuids::uuid source,
		                                   boost::uuids::uuid target,
		                                   const Transform& relative_pose,
		                                   const Covariance& covariance,
		                                   const std::string& sensor) = 0;

		/**
		 * @brief Get the current pose of the robot within the generated map.
		 * @details The pose is updated at least whenever a new node is added.
		 * Depending on the available information, it might be updated
		 * more often. (e.g. when odometry is available)
		 * @return current robot pose in map coordinates
		 */
		Transform getCurrentPose();
		
		/**
		 * @brief Start the backend optimization process.
		 * @details Requires that a Solver has been set with setSolver.
		 * @return true if optimization was successful
		 */
		virtual bool optimize() = 0;

		/**
		 * @brief Get the last vertex, that was locally added to the graph.
		 * @details This will not return external vertices from other robots.
		 * @return last added vertex
		 */
		virtual const VertexObject& getLastVertex() const = 0;

		/**
		 * @brief Write the current graph to a file (currently dot).
		 * @details For larger graphs, this can take a very long time.
		 * @param name filename without type ending
		 */
		virtual void writeGraphToFile(const std::string &name);

		/**
		 * @brief Sets neighbor radius for matching
		 * @details New nodes are matched against nodes of the same sensor
		 * within the given radius, but not more then given maximum.
		 * @param r radius within additional edges are created
		 * @param l maximum number of neighbor links
		 */
		void setNeighborRadius(float r, int l){ mNeighborRadius = r; mMaxNeighorLinks = l; }

		/**
		 * @brief Set minimal change in pose between adjacent nodes.
		 * @param t Minimum translation between nodes (in meter).
		 * @param r Minimum rotation between nodes (in rad).
		 */
		void setMinPoseDistance(float t, float r){ mMinTranslation = t; mMinRotation = r; }

		/**
		 * @brief Set how far to continue with a breadth-first-search through
		 * the pose graph when building local map patches to match new
		 * measurements against. It will use all vertices that are reachable
		 * by a maximum of r edges.
		 * @param r 
		 */
		void setPatchBuildingRange(unsigned r) {mPatchBuildingRange = r;}

		/**
		 * @brief Gets a vertex object by its given id. The id is given to each
		 * vertex upon creation and then never changed. These id's are local and
		 * cannot be compared between different agents in a distributed setup.
		 * @param id identifier for a vertex
		 * @return constant reference to a vertex
		 */
		virtual const VertexObject& getVertex(IdType id) const = 0;

		/**
		 * @brief Gets a vertex by the uuid of the attached Measurement.
		 * @param id uuid of a measurement
		 * @return constant reference to a vertex
		 */
		virtual const VertexObject& getVertex(boost::uuids::uuid id) const = 0;

		/**
		 * @brief 
		 * @param source
		 * @param target
		 * @param sensor
		 */
		virtual const EdgeObject& getEdge(IdType source, IdType target, const std::string& sensor) const = 0;

		/**
		 * @brief Get all outgoing edges from given source.
		 * @param source
		 */
		virtual EdgeObjectList getOutEdges(IdType source) const = 0;

		/**
		 * @brief Gets a list of all vertices from given sensor.
		 * @param sensor
		 */
		virtual VertexObjectList getVertexObjectsFromSensor(const std::string& sensor) const = 0;

		/**
		 * @brief Gets a list of all edges from given sensor.
		 * @param sensor
		 */
		virtual EdgeObjectList getEdgeObjectsFromSensor(const std::string& sensor) const = 0;

	protected:
		static Transform orthogonalize(const Transform& t);
		bool checkMinDistance(const Transform &t);
		
	protected:
		Solver* mSolver;
		Logger* mLogger;
		Odometry* mOdometry;
		SensorList mSensors;

		Transform mCurrentPose;
		Transform mLastOdometricPose;

		// Parameters
		int mMaxNeighorLinks;
		float mNeighborRadius;
		float mMinTranslation;
		float mMinRotation;
		bool mAddOdometryEdges;
		unsigned mPatchBuildingRange;
	};
}

#endif
