#ifndef SLAM3D_Neo4jGRAPH_HPP
#define SLAM3D_Neo4jGRAPH_HPP


#include <slam3d/core/Graph.hpp>

#include <boost/thread/shared_mutex.hpp>
#include <memory>


namespace web{ namespace http{ namespace client{ class http_client;}}}

namespace slam3d
{

	/**
	 * @class Neo4jGraph
	 * @brief Implementation of Graph using Neo4jGraphGraphLibrary.
	 */
	class Neo4jGraph : public Graph
	{
	public:
		Neo4jGraph(Logger* log);
		~Neo4jGraph();

		/**
		 * @brief delete the database contents
		 * @details deletes all Nodes and edges from the graph
		 * @return true if deletion was successful
		 */
		bool deleteDatabase();

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
		const VertexObject& getVertex(IdType id);
		
		/**
		 * @brief 
		 * @param source
		 * @param target
		 * @param sensor
		 */
		const EdgeObject& getEdge(IdType source, IdType target, const std::string& sensor);
		
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

		/**
		 * @brief Gets a list of all edges from given sensor.
		 * @param sensor
		 */
		EdgeObjectList getEdgesFromSensor(const std::string& sensor) const;
		
		/**
		 * @brief Get all connecting edges between given vertices.
		 * @param vertices
		 */
		EdgeObjectList getEdges(const VertexObjectList& vertices) const;

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

	protected:
		/**
		 * @brief Add the given VertexObject to the internal graph.
		 * @param v
		 */
		void addVertex(const VertexObject& v);
		
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
		 * @brief Get a writable reference to a VertexObject.
		 * @param id
		 */
		virtual VertexObject& getVertexInternal(IdType id);
		
		/**
		 * @brief Get a writable reference to an EdgeObject.
		 * @param source
		 * @param target
		 * @param sensor
		 */
		virtual EdgeObject& getEdgeInternal(IdType source, IdType target, const std::string& sensor);
		
		// /**
		//  * @brief 
		//  * @param source
		//  * @param target
		//  * @param sensor
		//  */
		// OutEdgeIterator getEdgeIterator(IdType source, IdType target, const std::string& sensor) const;

	private:

		std::string createQuery(const std::string& query, const std::vector<std::string>& params = std::vector<std::string>());

		// neo4j_connection_t *connection;
		// The boost graph object
		// AdjacencyGraph mPoseGraph;
		
		// Mutex for graph access
		mutable boost::shared_mutex mGraphMutex;
		
		// Index to map a vertex' id to its descriptor
		// IndexMap mIndexMap;

		//todo remove this and pass shared_ptr
		std::vector<VertexObject> vertexObjects;

		std::shared_ptr<web::http::client::http_client> client;

	};
}

#endif
