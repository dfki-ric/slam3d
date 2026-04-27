#pragma once

#include <string>
#include <functional>

#include <slam3d/core/Graph.hpp>


namespace slam3d
{
	class GraphSerialization
	{
	public:

		enum CloudMode {PORTABLE, BINARY, SKIP};

		/**
		 * @brief saves the graph to a folder
		 *
		 * @param graph the slam3d graph
		 * @param targetfolder folder to save the graph to
		 * @param graphfile file name of the exported graph
		 * @param status callback to give the processing status: void (current coud, total clouds)
		 * @param binaryClouds wheter to use text archives if binary
		 */
		static bool toFolder(Graph* graph,
			const std::string& targetfolder = ".",
			const std::string &graphfile = "slam3d_graph.yml",
			std::function<void(size_t,size_t)> status = nullptr,
			const CloudMode &cloudmode = PORTABLE);

		/**
		 * @brief restores the graph from folder
		 *
		 * @param graph the slam3d graph
		 * @param targetfolder folder to load the graph from
		 * @param graphfile file name of the exported graph
		 * @param status callback to give the processing status: void (current coud, total clouds)
		 * @param binaryClouds wheter to use text archives if binary
		 */
		static bool fromFolder(Graph* graph,
			const std::string& targetfolder = ".",
			const std::string &graphfile = "slam3d_graph.yml",
			std::function<void(size_t,size_t)> status = nullptr,
			const CloudMode &cloudmode = PORTABLE);

	};

}  // namespace slam3d
