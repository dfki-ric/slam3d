#pragma once

#include <string>
#include <functional>

#include "../core/Graph.hpp"



namespace slam3d {

class GraphSerialization {
 public:

    static bool toFolder(Graph& graph, const std::string targetfolder, std::function<void(size_t,size_t)> status = nullptr, bool binaryClouds = false);

    static bool fromFolder(Graph* graph, const std::string targetfolder, std::function<void(size_t,size_t)> status = nullptr, bool binaryClouds = false);

};

}  // namespace slam3d
