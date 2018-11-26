// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2017 S. Kasperski
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SLAM_GRAPH_HPP
#define SLAM_GRAPH_HPP

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
 * The central component of this library is the Graph class.
 * The documentation is best read by starting from there.
 * This class is extended by registering Sensor modules, PoseSensor modules and a Solver.
 * 
 * @section sec_example Programming example
 * 
 * Start by creating the graph itself and registering the required modules.
 @code
#include <slam3d/Graph.hpp> 
#include <slam3d/G2oSolver.hpp>

using namespace slam3d;
Clock* clock = new Clock();
Logger* logger = new Logger(*c);
Graph* graph = new Graph(logger);

PointCloudSensor* laser = new PointCloudSensor("laser", logger, Transform::Identity());
graph->registerSensor(laser);

G2oSolver* g2o = new G2oSolver(logger);
graph->setSolver(g2o);
 @endcode
 * Within the callback of your sensor data, add the new measurements to the corresponding sensor module.
 @code
Measurement m* = new PointCloudMeasurement(cloud, "my_robot", laser->getName(), laser->getSensorPose());
Transform odom = [...];
if(!laser->addReading(m, odom))
{
  delete m;
}
 @endcode
 */

#include "PoseSensor.hpp"
#include "Solver.hpp"

#include <flann/flann.hpp>
#include <map>

namespace slam3d
{
	typedef flann::Index< flann::L2<float> > NeighborIndex;
	
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
			msg << "Edge between " << source << " and " << target << " from sensor '" << sensor << "' already exists!";
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
	 * @class Graph
	 * @brief Holds measurements from different sensors in a graph.
	 * @details The Graph is the central structure that provides the
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
	class Graph
	{
	public:
		Graph(Logger* log);
		virtual ~Graph();

		/**
		 * @brief Sets a specific Solver to be used as SLAM backend.
		 * @details The mapper can be used without a backend,
		 * but mapping results might be inconsistant.
		 * @param solver backend to be used for optimization
		 */
		void setSolver(Solver* solver);

		/**
		 * @brief Add a given measurement at the given pose
		 * @details This method creates the VertexObject, adds the new vertex to
		 * the solver, adds it to the index and then calls the method below to
		 * actually add it to the graph.
		 * @param m measurement
		 * @param corrected initial pose for the new vertex
		 */
		IdType addVertex(Measurement::Ptr m, const Transform &corrected);

				/**
		 * @brief Add a constraint (edge) between two vertices in the graph.
		 * @param source
		 * @param target
		 * @param constraint
		 */
		virtual void addConstraint(IdType source,
		                           IdType target,
		                           Constraint::Ptr c) = 0;

		/**
		 * @brief Set the corrected pose for the vertex with the given ID.
		 * @details This method is designed to be used by Sensor and PoseSensor
		 * implementations in order to position newly added measurements.
		 * This allows to quickly set the pose of a vertex according to available
		 * odometry or scan matching results, before it is correctly estimated by
		 * executing the optimization backend.
		 * @param id vertex to be changed
		 * @param pose new corrected pose to be set
		 */
		void setCorrectedPose(IdType id, const Transform& pose);
		
		/**
		 * @brief Start the backend optimization process.
		 * @details Requires that a Solver has been set with setSolver.
		 * @param iterations maximum number of iteration steps
		 * @return true if optimization was successful
		 */
		virtual bool optimize(unsigned iterations = 100);
		
		/**
		 * @brief Returns whether optimize() has been called since the last call to this.
		 * @return true if optimization has been called
		 */
		bool optimized();

		/**
		 * @brief Write the current graph to a file (currently dot).
		 * @details For larger graphs, this can take a very long time.
		 * @param name filename without type ending
		 */
		virtual void writeGraphToFile(const std::string &name);

		/**
		 * @brief Create the index for nearest neighbor search of nodes.
		 * @param sensor index nodes of this sensor
		 */
		void buildNeighborIndex(const std::string& sensor);
		
		/**
		 * @brief Search for nodes in the graph near the given pose.
		 * @details This does not refer to a NN-Search in the graph, but to search for
		 * spatially near poses according to their current corrected pose.
		 * If new nodes have been added, the index has to be created with
		 * a call to buildNeighborIndex.
		 * @param tf The pose where to search for nodes
		 * @param radius The radius within nodes should be returned
		 * @return list of spatially near vertices
		 */
		VertexObjectList getNearbyVertices(const Transform &tf, float radius) const;

		/**
		 * @brief Gets the index of the vertex with the given Measurement
		 * @param id uuid of a measurement
		 */
		IdType getIndex(boost::uuids::uuid id) const;

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
		const VertexObject& getVertex(boost::uuids::uuid id) const;

		/**
		 * @brief Check if the measurement with this id is stored in the graph.
		 * @param id
		 */
		bool hasMeasurement(boost::uuids::uuid id) const;

		/**
		 * @brief Get the transformation between source and target node.
		 * @param source
		 * @param target
		 */
		TransformWithCovariance getTransform(IdType source, IdType target) const;

		/**
		 * @brief Get the edge between source and traget from the given sensor.
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
		virtual VertexObjectList getVerticesFromSensor(const std::string& sensor) const = 0;

		/**
		 * @brief Serch for nodes by using breadth-first-search
		 * @param source start search from this node
		 * @param range maximum number of steps to search from source
		 */
		virtual VertexObjectList getVerticesInRange(IdType source, unsigned range) const = 0;

		/**
		 * @brief Gets a list of all edges from given sensor.
		 * @param sensor
		 */
		virtual EdgeObjectList getEdgesFromSensor(const std::string& sensor) const = 0;

		/**
		 * @brief Get all connecting edges between given vertices.
		 * @param vertices
		 */
		virtual EdgeObjectList getEdges(const VertexObjectList& vertices) const = 0;
		
		/**
		 * @brief Calculates the distance between two vertices in the graph.
		 * @param source
		 * @param target
		 */
		virtual float calculateGraphDistance(IdType source, IdType target) = 0;
		
	protected:
		// Graph access
		/**
		 * @brief Add the given VertexObject to the actual graph.
		 * @details This method has to be implemented by the specification class.
		 * It should not be used directly, but is used internally.
		 * @param v VertexObject to be stored in the graph
		 */
		virtual void addVertex(const VertexObject& v) = 0;

		/**
		 * @brief Get a writable reference to a VertexObject.
		 * @param id
		 */
		virtual VertexObject& getVertexInternal(IdType id) = 0;
		
		// Helper
		/**
		 * @brief Re-orthogonalize the rotation-matrix
		 * @param t input tranform
		 * @return the orthogonalized transform
		 */
		static Transform orthogonalize(const Transform& t);
		
	protected:
		Solver* mSolver;
		Logger* mLogger;
		
		Indexer mIndexer;
		
		// Index to find Vertices by the unique id of their measurement
		typedef std::map<boost::uuids::uuid, IdType> UuidIndex;
		UuidIndex mUuidIndex;

		// Index to use nearest neighbor search
		// Whenever this index is created, we have to enumerate all vertices from 0 to n-1.
		// This mapping is kept in a separate map to later apply the result to the graph.
		flann::SearchParams mSearchParams;
		NeighborIndex mNeighborIndex;
		std::map<IdType, IdType> mNeighborMap; // vertex-id --> neighbor-id

		// Parameters
		bool mOptimized;
	};
}

#endif
