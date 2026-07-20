#include "GraphSerialization.hpp"


#include <boost/uuid/uuid_generators.hpp>
#include <boost/lexical_cast.hpp>

#include <slam3d/core/Graph.hpp>
#include "YamlTypes.hpp"

using namespace slam3d;


bool GraphSerialization::toFile(Graph* graph, const std::string &graphfile)
{
	auto &config = Yaml<YamlGraph>::getInstance();
	config.get().vertices = graph->getVertices();
	config.get().edges = graph->getEdges();
	return config.saveConfig(graphfile);
}

bool GraphSerialization::fromFile(Graph* graph, const std::string &graphfile)
{
	auto &config = Yaml<YamlGraph>::getInstance();
	config.loadConfig(graphfile);

	IdType maxId = 0;
	for (const auto& vertex : config.get().vertices)
	{
		graph->addVertex(vertex);
		if(vertex.index > maxId)
			maxId = vertex.index;
		graph->mUuidIndex[vertex.measurement.uniqueId] = vertex.index;
	}
	graph->mIndexer = Indexer(maxId+1);

	for (const auto& edge : config.get().edges)
	{
		graph->addEdge(edge);
	}

	// optimize locations
	graph->reloadToSolver();
	graph->optimize();
	return true;
}
