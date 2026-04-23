#include "GraphSerialization.hpp"


#include <boost/uuid/uuid_generators.hpp>
#include <boost/lexical_cast.hpp>

#include <slam3d/core/Graph.hpp>
#include "MeasurementSerialization.hpp"
#include "YamlTypes.hpp"

using namespace slam3d;


bool GraphSerialization::toFolder(Graph& graph, const std::string targetfolder, std::function<void(size_t,size_t)> status, bool binaryClouds) {
    
    
    auto &config = Yaml<YamlGraph>::getInstance();
    config.get().vertices.clear();

    size_t count = 0;
    slam3d::VertexObjectList vertices = graph.getVertices();
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


        newvertex.filename = targetfolder + "/" + std::to_string(vertex.index) + ".s3dm";

        slam3d::Measurement::Ptr m = graph.getMeasurement(vertex.measurementUuid);
        if (m) {
            MeasurementSerialization::toFile(m,newvertex.filename, binaryClouds);
        }

        slam3d::EdgeObjectList edges = graph.getOutEdges(vertex.index);
        for (const auto& edge : edges) {
            newvertex.children.push_back(edge);
        }
        config.get().vertices.push_back(newvertex);
    }
    
    config.saveConfig(targetfolder + "/" +"slam3dGraph.yml");

    return true;

}

bool GraphSerialization::fromFolder(Graph* graph, const std::string targetfolder, std::function<void(size_t,size_t)> status, bool binaryClouds) {

    std::string graphfile = targetfolder + "/slam3dGraph.yml";
    printf("loading Graph from file: %s\n",graphfile.c_str());

    auto &config = Yaml<YamlGraph>::getInstance();
    config.loadConfig(graphfile);

    graph->fixNext();

    std::map<size_t, size_t> newVertexId;
    
    std::map<size_t, YamlVertex> vertices; // use map to sort by vertex id
    for (const auto& vertex : config.get().vertices) {
        vertices[vertex.vertexIndex] = vertex;
    }

    // load vertices first
    for (const auto& vertex : vertices) {



        slam3d::Transform pose = vertex.second.correctedPose;
        slam3d::Transform sensorpose = vertex.second.sensorPose;

        boost::uuids::uuid uuid;
        if (vertex.second.measurementUuid == "") {
            uuid = boost::uuids::random_generator()();
        } else {
            uuid = boost::lexical_cast<boost::uuids::uuid>(vertex.second.measurementUuid);
        }

        slam3d::Measurement::Ptr measurement = MeasurementSerialization::fromFile(targetfolder + "/" + vertex.second.filename, binaryClouds);

        size_t vertexid = graph->addVertex(measurement, pose);
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
            graph->setCorrectedPose(target, vertex.correctedPose);    
        }
    }

    // optimize locations
    graph->reloadToSolver();
    graph->optimize();

    return true;
}