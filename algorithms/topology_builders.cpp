#include "contraction.h"
#include "topology_builders.h"

#include <execution>
#include <iostream>
#include <vector>

CompactTopology TopGraph::BuildTopology() {
    CompactTopology topology;
    topology.parents_.resize(parents_.size());
    for (auto [child, parent] : parents_) {
        topology.parents_.at(child) = parent;
    }
    std::transform_exclusive_scan(
        std::execution::par,
        layers_.begin(),
        layers_.end(),
        std::back_inserter(topology.sizes_),
        0,
        std::plus<>{},
        [](const auto& layer) {
            return layer.size();
        }
    );
    return topology;
}

std::pair<Graph, CompactTopology> BuildSimplyTopology(const Graph& graph, LevelId levels) {
    TopGraph topGraph(graph, levels);
    for (LevelId level = 1; level < levels; ++level) {
        std::unordered_set<VertexId> contracted;
        for (VertexId center : topGraph.Vertices(level - 1)) {
            if (contracted.contains(center)) {
                continue;
            }
            contracted.insert(center);
            auto cellId = topGraph.AddVertex(center, level);
            for (EdgeId edgeId : topGraph.GetOutgoingEdges(center)) {
                auto to = topGraph.GetTarget(edgeId);
                if (!contracted.contains(to)) {
                    contracted.insert(to);
                    topGraph.SetCell(to, cellId);
                }
            }
        }

        for (VertexId from : topGraph.Vertices(level - 1)) {
            for (EdgeId edgeId : topGraph.GetOutgoingEdges(from)) {
                auto to = topGraph.GetTarget(edgeId);
                if (topGraph.GetCellId(from) != topGraph.GetCellId(to)) {
                    topGraph.AddEdge(topGraph.GetCellId(from), topGraph.GetCellId(to), topGraph.GetEdgeProperties(edgeId));
                }
            }
        }
    }

    return std::make_pair(topGraph.builder_.Build(), topGraph.BuildTopology());
}
