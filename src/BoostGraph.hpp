#ifndef SLAM3D_BOOSTGRAPH_HPP
#define SLAM3D_BOOSTGRAPH_HPP

#include "Graph.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

namespace slam3d
{
	// Definitions of boost-graph related types
	typedef boost::listS VRep;
	typedef boost::vecS ERep;
	typedef boost::directedS GType;
	typedef boost::adjacency_list<VRep, ERep, GType, VertexObject, EdgeObject> AdjacencyGraph;
	
	typedef boost::graph_traits<AdjacencyGraph>::vertex_descriptor Vertex;
	typedef boost::graph_traits<AdjacencyGraph>::vertex_iterator VertexIterator;
	typedef std::pair<VertexIterator, VertexIterator> VertexRange;
	
	typedef boost::graph_traits<AdjacencyGraph>::edge_descriptor Edge;
	typedef boost::graph_traits<AdjacencyGraph>::edge_iterator EdgeIterator;
	typedef boost::graph_traits<AdjacencyGraph>::out_edge_iterator OutEdgeIterator;
	typedef boost::graph_traits<AdjacencyGraph>::in_edge_iterator InEdgeIterator;
	typedef std::pair<EdgeIterator, EdgeIterator> EdgeRange;
	
	typedef boost::graph_traits<AdjacencyGraph>::adjacency_iterator AdjacencyIterator;
	typedef std::pair<AdjacencyIterator, AdjacencyIterator> AdjacencyRange;
	
	// List types
	typedef std::vector<Vertex> VertexList;
	typedef std::vector<Edge> EdgeList;

	// Index types
	typedef std::map<IdType, Vertex> IndexMap;
	
	/**
	 * @class BoostGraph
	 * @brief Implementation of Graph using BoostGraphLibrary.
	 */
	class BoostGraph : public Graph
	{
	public:
		BoostGraph(Logger* log);
		~BoostGraph();

		/**
		 * @brief Add a new measurement to the graph.
		 * @details The sensor specified in the measurement has to be registered
		 * with the mapper before. If the change in robot pose since the last
		 * added scan is smaller then min-translation or min-rotation, the
		 * measurement will not be added. Use Graph::setMinPoseDistance to
		 * adjust this distance.
		 * @param m pointer to a new measurement
		 * @param force add measurement regardless of change in robot pose
		 * @return true if the measurement was added
		 */
		IdType addReading(Measurement::Ptr m);

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
		 * @throw DuplicateMeasurement
		 */
		void addExternalReading(Measurement::Ptr measurement,
		                        boost::uuids::uuid source_uuid,
		                        const Transform& tf,
		                        const Covariance& cov,
		                        const std::string& sensor);

		/**
		 * @brief Adds a new edge to the graph.
		 * @param source id of source vertex
		 * @param target id of target vertex
		 * @param t transformation from source to target
		 * @param c covariance of transformation
		 * @param sensor name of the sensor that created this edge
		 * @param label description to be added to this edge
		 * @return 
		 */
		void addConstraint(IdType source,
		                   IdType target,
		                   const Transform& relative_pose,
		                   const Covariance& covariance,
		                   const std::string& sensor,
		                   const std::string& label);
		
		/**
		 * @brief 
		 * @param v
		 */
		void addVertex(const VertexObject& v);
		
		/**
		 * @brief Get the last vertex, that was locally added to the graph.
		 * @details This will not return external vertices from other robots.
		 * @return last added vertex
		 */
		const VertexObject& getLastVertex() const;
		
		/**
		 * @brief Start the backend optimization process.
		 * @details Requires that a Solver has been set with setSolver.
		 * @return true if optimization was successful
		 */
		bool optimize();
		
		/**
		 * @brief Gets a vertex object by its given id.
		 * @param id
		 * @throw std::out_of_range
		 */
		const VertexObject& getVertex(IdType id) const;

		bool hasMeasurement(boost::uuids::uuid id) const;

		/**
		 * @brief Gets the edge from given sensor between source and target.
		 * @param source
		 * @param target
		 * @param sensor
		 * @throw std::out_of_range if source or target don't exist
		 * @throw InvalidEdge
		 */
		const EdgeObject& getEdge(IdType source, IdType target, const std::string& sensor) const;

		/**
		 * @brief Get all outgoing edges from given source.
		 * @param source
		 * @throw std::out_of_range
		 */
		EdgeObjectList getOutEdges(IdType source) const;
		
		/**
		 * @brief Gets a list of all vertices from given sensor.
		 * @param sensor
		 */
		VertexObjectList getVerticesFromSensor(const std::string& sensor) const;

		/**
		 * @brief Serch for nodes by using breadth-first-search
		 * @param source start search from this node
		 * @param range maximum number of steps to search from source
		 */
		VertexObjectList getVerticesInRange(IdType source, unsigned range) const;

//		VertexObjectList getNearbyUnlinkedVertices(const Transform &tf, float radius, const std::string &sensor) const;

		/**
		 * @brief Gets a list of all edges from given sensor.
		 * @param sensor
		 */
		EdgeObjectList getEdgesFromSensor(const std::string& sensor) const;
		
		/**
		 * @brief Get all connecting edges between given vertices.
		 * @param vertices
		 */
		EdgeObjectList getEdges(const VertexObjectList& vertices);

		/**
		 * @brief Calculates the distance between two vertices in the graph.
		 * @param source
		 * @param target
		 */
		float calculateGraphDistance(IdType source, IdType target);
		
		/**
		 * @brief Write the current graph to a file (currently dot).
		 * @details For larger graphs, this can take a very long time.
		 * @param name filename without type ending
		 */
		void writeGraphToFile(const std::string &name);
		
	private:
		
		/**
		 * @brief Gets a list with all vertices from a given sensor.
		 * @param sensor name of the sensor which vertices are requested
		 * @return list of all vertices from given sensor
		 */
		VertexList getVerticesFromSensor(const std::string& sensor);

		/**
		 * @brief Get a list with all edges from a given sensor.
		 * @param sensor name of the sensor which edges are requested
		 * @return list of all edges from given sensor
		 */
		EdgeList getEdgesFromSensor(const std::string& sensor);
		
	private:
		// The boost graph object
		AdjacencyGraph mPoseGraph;
		
		// Index to map a vertex' id to its descriptor
		IndexMap mIndexMap;
	};
}

#endif
