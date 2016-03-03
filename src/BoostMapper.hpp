#ifndef SLAM3D_BOOSTMAPPER_HPP
#define SLAM3D_BOOSTMAPPER_HPP

#include "GraphMapper.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <flann/flann.hpp>

namespace slam3d
{
	// Definitions of boost-graph related types
	typedef boost::listS VRep;
	typedef boost::listS ERep;
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
	typedef flann::Index< flann::L2<float> > NeighborIndex;
	typedef std::map<IdType, Vertex> IndexMap;
	typedef std::map<boost::uuids::uuid, Vertex> UuidMap;
	
	/**
	 * @class BoostMapper
	 * @brief Implementation of GraphMapper using BoostGraphLibrary.
	 */
	class BoostMapper : public GraphMapper
	{
	public:
		BoostMapper(Logger* log);
		~BoostMapper();

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
		bool addReading(Measurement::Ptr m);

		/**
		 * @brief Add a new measurement from another robot.
		 * @param m pointer to a new measurement
		 * @param t pose in map coordinates
		 */
		void addExternalReading(Measurement::Ptr m, const Transform& t);
		
		void addExternalConstraint(boost::uuids::uuid source,
		                           boost::uuids::uuid target,
		                           const Transform& relative_pose,
		                           const Covariance& covariance,
		                           const std::string& sensor);
										   
		/**
		 * @brief Get the last vertex, that was locally added to the graph.
		 * @details This will not return external vertices from other robots.
		 * @return last added vertex
		 */
		const VertexObject& getLastVertex() { return mPoseGraph[mLastVertex]; }
		
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
		const VertexObject& getVertex(IdType id);

		/**
		 * @brief 
		 * @param source
		 * @param target
		 * @param sensor
		 * @throw std::out_of_range, InvalidEdge
		 */
		const EdgeObject& getEdge(IdType source, IdType target, const std::string& sensor);

		/**
		 * @brief Get all outgoing edges from given source.
		 * @param source
		 * @throw std::out_of_range
		 */
		virtual EdgeObjectList getOutEdges(IdType source) const;
		
		/**
		 * @brief Gets a list of all vertices from given sensor.
		 * @param sensor
		 */
		VertexObjectList getVertexObjectsFromSensor(const std::string& sensor);

		/**
		 * @brief Gets a list of all edges from given sensor.
		 * @param sensor
		 */
		EdgeObjectList getEdgeObjectsFromSensor(const std::string& sensor);

	private:
	
		/**
		 * @brief Adds a new vertex to the graph.
		 * @param m measurement to be attached to the vertex
		 * @param corrected initial pose of the vertex in map coordinates
		 * @return descriptor of the new vertex
		 */
		Vertex addVertex(Measurement::Ptr m,
		                 const Transform &corrected);

		/**
		 * @brief Adds a new edge to the graph.
		 * @param source descriptor of source vertex
		 * @param target descriptor of target vertex
		 * @param t transformation from source to target
		 * @param c covariance of transformation
		 * @param sensor name of the sensor that created this edge
		 * @param label description to be added to this edge
		 * @return 
		 */
		void addEdge(Vertex source,
		             Vertex target,
		             const Transform &t,
		             const Covariance &c,
		             const std::string &sensor,
		             const std::string &label);

		/**
		 * @brief 
		 * @param source
		 * @param target
		 * @param sensor
		 */
		TransformWithCovariance link(Vertex source, Vertex target, Sensor* sensor);

		void linkToNeighbors(Vertex vertex, Sensor* sensor, int max_links);
		
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
		VertexList getNearbyVertices(const Transform &tf, float radius);
		
		/**
		 * @brief Serch for nodes by using breadth-first-search
		 * @param source start search from this node
		 * @param range maximum number of steps to search from source
		 */
		VertexList getVerticesInRange(Vertex source, unsigned range);
		
	private:
		// The boost graph object
		AdjacencyGraph mPoseGraph;
		Indexer mIndexer;
		
		// Index to map a vertex' id to its descriptor
		IndexMap mIndexMap;
		
		// Index to use nearest neighbor search
		// Whenever this index is created, we have to enumerate all vertices from 0 to n-1.
		// This mapping is kept in a separate map to later apply the result to the graph.
		flann::SearchParams mSearchParams;
		NeighborIndex mNeighborIndex;
		IndexMap mNeighborMap;

		// Index to find Vertices by their unique id
		UuidMap mVertexIndex;
		
		// Some special vertices
		Vertex mLastVertex;
		Vertex mFirstVertex;
	};
}

#endif