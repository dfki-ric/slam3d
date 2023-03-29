#ifndef SLAM3D_Neo4jGRAPH_HPP
#define SLAM3D_Neo4jGRAPH_HPP

#include <memory>
#include <string>
#include <vector>

// #include <boost/thread/shared_mutex.hpp>

#include "../../core/Graph.hpp"


// dont defiene the U macro in the http client, it conflicts with the eigen U macro use _XPLATSTR() instead
#define _TURN_OFF_PLATFORM_STRING
#include <cpprest/json.h>
#include <cpprest/http_client.h>

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

        class Query {
         public:
            Query(std::shared_ptr<web::http::client::http_client> client):client(client) {
                // query = web::json::value::object(true); sorted
                query["statements"] = web::json::value::array(1);
                query["statements"][0]["parameters"] = web::json::value();
                // printf("%s:%i \n\t%s\n", __PRETTY_FUNCTION__, __LINE__, query.serialize().c_str());
            }

            void setStatement(const std::string& statement) {
                query["statements"][0]["statement"] = web::json::value(statement);
                // printf("%s:%i \n\t%s\n", __PRETTY_FUNCTION__, __LINE__, query.serialize().c_str());
            }

            void addParameterSet(const std::string& setname) {
                query["statements"][0]["parameters"][setname] = web::json::value();
                // printf("%s:%i \n\t%s\n", __PRETTY_FUNCTION__, __LINE__, query.serialize().c_str());
            }

            template <class VALUE> void addParameterToSet(const std::string& setname, const std::string& paramname, const VALUE& value) {
                query["statements"][0]["parameters"][setname][paramname] = web::json::value(value);
            }

            web::json::value* getParameterSet(const std::string& setname) {
                if (query["statements"][0]["parameters"][setname].is_null()) {
                    addParameterSet(setname);
                }
                return &(query["statements"][0]["parameters"][setname]);
            }

            bool sendQuery() {
                response = web::http::http_response();
                printf("%s:%i \n\t%s\n", __PRETTY_FUNCTION__, __LINE__, query.serialize().c_str());
                response = client->request(web::http::methods::POST, "/db/neo4j/tx/commit", query.serialize(), "application/json;charset=utf-8").get();
                if (response.status_code() != 200) {
                    std::cout << "ERROR " << response.to_string() << std::endl;
                    return false;
                }
                return true;
            }

            web::http::http_response& getResponse() {
                return response;
            }

         private:
            std::shared_ptr<web::http::client::http_client> client;
            web::json::value query;
            web::http::http_response response;
        };

        explicit Neo4jGraph(Logger* log);
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
        EdgeObjectList getEdgesFromSensor(const std::string& sensor);

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
        std::string createQuery(const std::string& query, const web::json::value& params = web::json::value());


        void constraintToJson(slam3d::Constraint::Ptr constraint, web::json::value* json);
        slam3d::Constraint::Ptr jsonToConstraint(web::json::value& json);

        std::string eigenMatrixToString(const Eigen::MatrixXd& mat);
        Eigen::MatrixXd eigenMatrixFromString(const std::string & string);

        slam3d::EdgeObject edgeObjectFromJson(web::json::value& json);

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
};
}

#endif
