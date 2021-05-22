#pragma once

#include "contraction.h"
#include "topology.h"

WGraph BuildTestGraph();

CoordGraph BuildTestCoordGraph();

IntermediateGraph PreprocessGraph(
    const WGraph& originalGraph,
    const std::filesystem::path& path,
    const Topology& topology);
