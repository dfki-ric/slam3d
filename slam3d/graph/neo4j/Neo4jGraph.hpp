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

namespace web{ 
    namespace http{ namespace client{ class http_client;}}
    namespace json{ class value;}
}

namespace slam3d {
    /**
     * @class Neo4jGraph
     * @brief Implementation of Graph using Neo4jGraphGraphLibrary.
     */
class Neo4jGraph : public Graph {
    public:

        // class Query : public web::json::value {
        //     Query() {
        //         (*this)["statements"] = web::json::array();

        //     };

        // void add:

    // std::string json = "{ \"statements\": [{\"statement\": \""+query+"\"";

    // json += ", \"parameters\": ";
    // // if (!params.is_null()) {
    //     json += params.serialize();
    // // }
    // // json += "}";

    // json += "}]}";
    // return json;


        // };

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
        std::string createQuery(const std::string& query, const web::json::value& params = web::json::value());


        void constraintToJSON(slam3d::Constraint::Ptr constraint, web::json::value* json);

        static std::string eigenMatrixToString(const Eigen::MatrixXd& mat) {
            std::stringstream ss;
            Eigen::IOFormat jsonfmt(Eigen::FullPrecision, 0, ", ", ", ", "[", "]", "[", "]");
            ss << mat.format(jsonfmt);
            return ss.str();
        }

        static slam3d::Transform slam3dTransformFromString(const std::string & string) {
            
            printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
            std::cout << string << std::endl;
            web::json::value val = web::json::value::parse(string);
            printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
            std::cout << val << std::endl;

            Eigen::Matrix4d mat;

            mat << val[0][0].as_double(), val[0][1].as_double(), val[0][2].as_double(), val[0][3].as_double(), \
                   val[1][0].as_double(), val[1][1].as_double(), val[1][2].as_double(), val[1][3].as_double(), \
                   val[2][0].as_double(), val[2][1].as_double(), val[2][2].as_double(), val[2][3].as_double(), \
                   val[3][0].as_double(), val[3][1].as_double(), val[3][2].as_double(), val[3][3].as_double();
            std::cout << mat << std::endl;
            return slam3d::Transform(mat);
        }

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
