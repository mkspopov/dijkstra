#pragma once

#include "contraction.h"
#include "multilevel_graph.h"

WGraph BuildTestGraph();

CoordGraph BuildTestCoordGraph();

IntermediateGraph PreprocessGraph(
    const WGraph& originalGraph,
    const std::filesystem::path& path,
    CompactTopology topology);
