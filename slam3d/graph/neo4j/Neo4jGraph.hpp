#ifndef SLAM3D_Neo4jGRAPH_HPP
#define SLAM3D_Neo4jGRAPH_HPP

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <slam3d/core/Graph.hpp>
#include <slam3d/core/MeasurementStorage.hpp>




#include "Neo4jQuery.hpp"
#include "Neo4jConversion.hpp"
// #include "RedisMap.hpp"
// #include <boost/thread/shared_mutex.hpp>


// namespace web{ 
//     namespace http{ namespace client{ class http_client;}}
//     namespace json{ class value;}
// }


namespace slam3d {
    /**
     * @class Neo4jGraph
     * @brief Implementation of Graph using Neo4jGraphGraphLibrary.
     */
class Neo4jGraph : public Graph {
    public:
        struct Server {
            Server(const std::string& host, const int &port, const std::string& user, const std::string& passwd) : host(host), port(port), user(user), passwd(passwd) {}
            const std::string& host;
            const int &port;
            const std::string& user;
            const std::string& passwd;
        };

        Neo4jGraph(Logger* log,
                    MeasurementStorage* storage,
                    const Server &graphserver = Neo4jGraph::Server("127.0.0.1", 7474, "neo4j", "neo4j")
                    );
        ~Neo4jGraph();

        /**
         * @brief delete the database contents
         * @details deletes all Nodes and edges from the graph
         * @return true if deletion was successful
         */
        bool deleteDatabase();

        /**
         * @brief send a query and run fuinctio for each returned element
         * 
         * @param query 
         * @param function 
         * @return true 
         * @return false 
         */
        bool runQuery(const std::string query, std::function<void (neo4j_result_t *element)> function) const;

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
        const VertexObject getVertex(IdType id) const;

        const VertexObject getVertex(boost::uuids::uuid id) const;

        virtual void setVertex(IdType id, const VertexObject& v);

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
		 * @brief Get a list of sensors of all vertices in the graph
		 * @return const std::set<std::string> list of all sensors within the graph
		 */
		virtual const std::set<std::string> getVertexSensors() const;

		/**
		 * @brief Get a list of sensors of all vertices in the graph
		 * @return const std::set<std::string> list of all sensors within the graph
		 */
		virtual const std::set<std::string> getEdgeSensors() const;

        /**
         * @brief Gets a list of all vertices from given sensor.
         * @param sensor
         */
        virtual const VertexObjectList getVerticesFromSensor(const std::string& sensor) const;

		/**
		 * @brief Gets a list of all vertices with a given measurement type.
		 * @param sensor
		 */
		virtual const VertexObjectList getVerticesByType(const std::string& type) const;

        /**
         * @brief Serch for nodes by using breadth-first-search
         * @param source start search from this node
         * @param range maximum number of steps to search from source
         */
        virtual const VertexObjectList getVerticesInRange(IdType source, unsigned range) const;


        /**
         * @brief override buildNeighborIndex as getNearbyVertices is evaluated by the neo4j database
         * 
         * @param sensors 
         */
		void buildNeighborIndex(const std::set<std::string>& sensors){}

		/**
		 * @brief Serch for nodes by location and radius
		 * @param location x,y,z location of the center
		 * @param radius the radius of the return
		 * @throw InvalidVertex
		 */
        virtual const VertexObjectList getNearbyVertices(const Transform &location, float radius, const std::string& sensortype = "") const;

        /**
         * @brief return lost of all Vertices in the graph (to accumulate a global map with different sources, i.e. not all sensor names are known)
         *
         * @return const VertexObjectList
         */
        const VertexObjectList getAllVertices() const;

        /**
         * @brief Gets a list of all edges from given sensor.
         * @param sensor
         */
        const EdgeObjectList getEdgesFromSensor(const std::string& sensor) const;

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
        float calculateGraphDistance(IdType source, IdType target) const;

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
         * @brief Add the given EdgeObject to the internal graph.
         * @param e
         */
        // version to overload graph interface
        virtual void addEdge(const EdgeObject& e);
        // internal version
        virtual void addEdge(const EdgeObject& e, bool addInverse);

        /**
         * @brief 
         * @param source
         * @param target
         * @param sensor
         */
        virtual void removeEdge(IdType source, IdType target, const std::string& sensor);

        // /**
        //  * @brief Get a writable reference to a VertexObject.
        //  * @param id
        //  */
        // virtual VertexObject& getVertexInternal(IdType id);

        // /**
        //  * @brief Get a writable reference to an EdgeObject.
        //  * @param source
        //  * @param target
        //  * @param sensor
        //  */
        // virtual EdgeObject& getEdgeInternal(IdType source, IdType target, const std::string& sensor);

        // // /**
        //  * @brief
        //  * @param source
        //  * @param target
        //  * @param sensor
        //  */
        // OutEdgeIterator getEdgeIterator(IdType source, IdType target, const std::string& sensor) const;

    private:
        std::string createQuery(const std::string& query, const web::json::value& params = web::json::value());



        // neo4j_connection_t *connection;
        // The boost graph object
        // AdjacencyGraph mPoseGraph;

        // Mutex for graph access
        // mutable boost::shared_mutex mGraphMutex;

        // Index to map a vertex' id to its descriptor
        // IndexMap mIndexMap;

        // todo remove this and pass shared_ptr
        std::vector<VertexObject> vertexObjects;

        std::shared_ptr<web::http::client::http_client> client;
        neo4j_connection_t *connection;


        // todo replace with databae (value store)
        // std::map<std::string, Measurement::Ptr> measurements;

        // std::shared_ptr<RedisMap> measurements;
        // std::shared_ptr<MeasurementStorage> measurements;


        slam3d::Logger* logger;

        mutable std::mutex queryMutex;
};
}  // namespace slam3d

#endif
