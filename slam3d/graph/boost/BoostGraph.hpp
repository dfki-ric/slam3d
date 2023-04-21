#ifndef SLAM3D_BOOSTGRAPH_HPP
#define SLAM3D_BOOSTGRAPH_HPP

#include <slam3d/core/Graph.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/thread/shared_mutex.hpp>

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
		 * @brief Start the backend optimization process.
		 * @details Requires that a Solver has been set with setSolver.
		 * @param iterations maximum number of iteration steps
		 * @return true if optimization was successful
		 */
		bool optimize(unsigned iterations = 100);
		
		/**
		 * @brief 
		 * @param id
		 */
		const VertexObject getVertex(IdType id);
		
		/**
		 * @brief 
		 * @param source
		 * @param target
		 * @param sensor
		 */
		const EdgeObject getEdge(IdType source, IdType target, const std::string& sensor) const;
		
		/**
		 * @brief Get all outgoing edges from given source.
		 * @param source
		 * @throw std::out_of_range
		 */
		const EdgeObjectList getOutEdges(IdType source) const;
		
		/**
		 * @brief Gets a list of all vertices from given sensor.
		 * @param sensor
		 */
		const VertexObjectList getVerticesFromSensor(const std::string& sensor);
		
		/**
		 * @brief Serch for nodes by using breadth-first-search
		 * @param source start search from this node
		 * @param range maximum number of steps to search from source
		 */
		const VertexObjectList getVerticesInRange(IdType source, unsigned range) const;

		/**
		 * @brief Gets a list of all edges from given sensor.
		 * @param sensor
		 */
		const EdgeObjectList getEdgesFromSensor(const std::string& sensor);
		
		/**
		 * @brief Get all connecting edges between given vertices.
		 * @param vertices
		 */
		const EdgeObjectList getEdges(const VertexObjectList& vertices) const;

		/**
		 * @brief Calculates the minimum number of edges between two vertices in the graph.
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

		void setCorrectedPose(IdType id, const Transform& pose);

	protected:
		/**
		 * @brief Add the given VertexObject to the internal graph.
		 * @param v
		 */
		void addVertex(const VertexObject& v);
		
		/**
		 * @brief Set the given VertexObject to the internal graph.
		 * @param id
		 * @param v
		 */
		void setVertex(IdType id, const VertexObject& v);
		
		/**
		 * @brief Add the given EdgeObject to the internal graph.
		 * @param e
		 */
		virtual void addEdge(const EdgeObject& e);
		
		/**
		 * @brief 
		 * @param source
		 * @param target
		 * @param sensor
		 */
		virtual void removeEdge(IdType source, IdType target, const std::string& sensor);

		
		/**
		 * @brief 
		 * @param source
		 * @param target
		 * @param sensor
		 */
		OutEdgeIterator getEdgeIterator(IdType source, IdType target, const std::string& sensor) const;

		/**
		 * @brief 
		 * 
		 * @param source_id 
		 * @param target_id 
		 * @param c 
		 */
		void replaceConstraint(IdType source_id, IdType target_id, Constraint::Ptr c);

	private:
		// The boost graph object
		AdjacencyGraph mPoseGraph;
		
		// Mutex for graph access
		mutable boost::shared_mutex mGraphMutex;
		
		// Index to map a vertex' id to its descriptor
		IndexMap mIndexMap;
	};
}

#endif
