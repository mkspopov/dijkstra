#pragma once

#include "graph.h"

#include <unordered_set>

struct MinCut {
    std::unordered_set<VertexId> first;
    std::unordered_set<VertexId> second;
    std::vector<EdgeId> edges;
};

MinCut FindMinCut(
    const std::vector<VertexId>& sources,
    const std::vector<VertexId>& sinks,
    const Graph& graph);
