#pragma once

#include <string>

#include <slam3d/core/Graph.hpp>


namespace slam3d
{
	class GraphSerialization
	{
	public:
		/**
		 * @brief saves the graph to a folder
		 *
		 * @param graph the slam3d graph
		 * @param graphfile file name of the exported graph
		 */
		static bool toFile(Graph* graph, const std::string &graphfile = "slam3d_graph.yml");

		/**
		 * @brief restores the graph from folder
		 *
		 * @param graph the slam3d graph
		 * @param graphfile file name of the exported graph
		 */
		static bool fromFile(Graph* graph, const std::string &graphfile = "slam3d_graph.yml");
	};

}  // namespace slam3d
