#include "GraphSerialization.hpp"


#include <boost/uuid/uuid_generators.hpp>
#include <boost/lexical_cast.hpp>

#include <slam3d/core/Graph.hpp>
#include "MeasurementSerialization.hpp"
#include "YamlTypes.hpp"

using namespace slam3d;


bool GraphSerialization::toFolder(Graph* graph, const std::string& targetfolder, const std::string &graphfile, std::function<void(size_t,size_t)> status, const CloudMode &cloudmode) {
    
    
    auto &config = Yaml<YamlGraph>::getInstance();
    config.get().vertices.clear();

    size_t count = 0;
    slam3d::VertexObjectList vertices = graph->getVertices();
    for (const auto& vertex : vertices) {

        if (status) {
            status(++count, vertices.size());
        }

        YamlVertex newvertex;
        newvertex.vertexIndex = vertex.index;

        newvertex.correctedPose = vertex.correctedPose;
        
        newvertex.robotName = vertex.robotName;
        newvertex.sensorName = vertex.sensorName;
        newvertex.typeName = vertex.typeName;

        newvertex.tv_sec = vertex.timestamp.tv_sec;
        newvertex.tv_usec = vertex.timestamp.tv_usec;

        newvertex.measurementUuid = boost::lexical_cast<std::string>(vertex.measurementUuid);
        newvertex.filename = std::to_string(vertex.index) + ".s3dm";


        if (cloudmode != SKIP) {
            slam3d::Measurement::Ptr m = graph->getMeasurement(vertex.measurementUuid);
            if (m) {
                if (cloudmode == PORTABLE){
                    MeasurementSerialization::toFile(m,targetfolder + "/" + newvertex.filename, false);
                } else if (cloudmode == BINARY){
                    MeasurementSerialization::toFile(m,targetfolder + "/" + newvertex.filename, true);
                }
            }
        }

        slam3d::EdgeObjectList edges = graph->getOutEdges(vertex.index);
        for (const auto& edge : edges) {
            newvertex.children.push_back(edge);
        }
        config.get().vertices.push_back(newvertex);
    }
    
    config.saveConfig(targetfolder + "/" + graphfile);

    return true;

}

bool GraphSerialization::fromFolder(Graph* graph, const std::string& targetfolder, const std::string &graphfile, std::function<void(size_t,size_t)> status, const CloudMode &cloudmode) {

    std::string graphfilefull = targetfolder + "/" + graphfile;
    printf("loading Graph from file: %s\n",graphfilefull.c_str());

    auto &config = Yaml<YamlGraph>::getInstance();
    config.loadConfig(graphfilefull);

    graph->fixNext();

    std::map<size_t, size_t> newVertexId;
    
    std::map<size_t, YamlVertex> vertices; // use map to sort by vertex id
    for (const auto& vertex : config.get().vertices) {
        vertices[vertex.vertexIndex] = vertex;
    }

    // load vertices first
    for (const auto& vertex : vertices) {

        boost::uuids::uuid uuid;
        if (vertex.second.measurementUuid == "") {
            uuid = boost::uuids::random_generator()();
        } else {
            uuid = boost::lexical_cast<boost::uuids::uuid>(vertex.second.measurementUuid);
        }
        slam3d::Measurement::Ptr measurement;

        if (cloudmode != SKIP) {
            std::string filename = targetfolder + "/" + vertex.second.filename;
            if (cloudmode == PORTABLE){
                measurement = MeasurementSerialization::fromFile(filename, false);
            } else if (cloudmode == BINARY){
                measurement = MeasurementSerialization::fromFile(filename, true);
            }
        }

        size_t vertexid = graph->addVertex(measurement, vertex.second.correctedPose);
        newVertexId[vertex.first] = vertexid;

        if (status) {
            status(vertexid,vertices.size());
        }

    }

    //go overr egdes and add with new vertex ids
    for (const auto& vertex : config.get().vertices) {
        for (const auto& edge : vertex.children) {
            size_t source = newVertexId[edge.source];
            size_t target = newVertexId[edge.target];

            graph->addConstraint(source, target, edge.constraint);
        }
    }

    // optimize locations
    graph->reloadToSolver();
    graph->optimize();

    return true;
}