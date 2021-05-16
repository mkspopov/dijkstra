#include "contraction.h"
#include "topology_builders.h"

#include <execution>
#include <iostream>
#include <vector>

struct TopGraph {
    TopGraph(WeightGraph<EdgeProperty> graph, LevelId levels)
        : builder_(std::move(graph))
        , parents_(levels + 1)
    {
        auto level0 = Range(static_cast<VertexId>(0), builder_.graph_.VerticesCount());
        layers_.emplace_back(level0.begin(), level0.end());
    }

    VertexId GetCellId(VertexId vertex) const {
        return parents_.at(vertex);
    }

    VertexId GetTarget(EdgeId edgeId) const {
        return builder_.graph_.GetTarget(edgeId);
    }

    auto GetOutgoingEdges(VertexId vertex) const {
        return builder_.graph_.GetOutgoingEdges(vertex);
    }

    const auto& Vertices(LevelId level) const {
        return layers_.at(level);
    }

    auto GetEdgeProperties(EdgeId edgeId) const {
        return builder_.graph_.GetEdgeProperties(edgeId);
    }

    CompactTopology BuildTopology() {
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

private:
    friend CompactTopology BuildSimplyTopology(const WeightGraph<EdgeProperty>& graph, LevelId levels);

    VertexId AddVertex(LevelId level) {
        auto id = builder_.AddVertex();
        if (layers_.size() <= level) {
            layers_.emplace_back();
        }
        layers_.at(level).push_back(id);
        return id;
    }

    void AddEdge(VertexId from, VertexId to, EdgeProperty edgeProperty) {
        builder_.AddEdge(from, to, std::move(edgeProperty));
    }

    void SetCell(VertexId vertexId, VertexId cellId) {
        auto [_, emplaced] = parents_.emplace(vertexId, cellId);
        ASSERT(emplaced);
    }

    GraphBuilder<WeightGraph<EdgeProperty>, EdgeProperty> builder_;
    std::vector<std::vector<VertexId>> layers_;
    std::unordered_map<VertexId, VertexId> parents_;
};

CompactTopology BuildSimplyTopology(const WeightGraph<EdgeProperty>& graph, LevelId levels) {
    TopGraph topGraph(graph, levels);
    for (LevelId level = 1; level < levels; ++level) {
        std::unordered_set<VertexId> contracted;
        for (VertexId center : topGraph.Vertices(level - 1)) {
            if (contracted.contains(center)) {
                continue;
            }
            contracted.insert(center);
            auto cellId = topGraph.AddVertex(level);
            topGraph.SetCell(center, cellId);
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

    ASSERT(!topGraph.layers_.back().empty());
    auto root = topGraph.AddVertex(levels);
    for (auto child : topGraph.layers_.at(levels - 1)) {
        topGraph.SetCell(child, root);
    }

    return topGraph.BuildTopology();
}

struct Partitioning {
    std::vector<VertexId> first;
    std::vector<VertexId> second;
};

//std::vector<EdgeId> Split() {
//
//}

//CompactTopology BuildTopologyInertialFlow(WeightGraph<EdgeProperty> graph, LevelId levels, int depth) {
//
//    if (depth == levels) {
//        return {};
//    }
//    auto cutEdges = FindCutEdges(graph);
//    for (auto edgeId : cutEdges) {
//        graph.GetEdges()[edgeId]
//    }
//}
//
//std::pair<Graph, CompactTopology> BuildTopologyInertialFlow(const WeightGraph<EdgeProperty>& graph, LevelId levels) {
//
//    return std::make_pair(graph, BuildTopologyInertialFlow(graph, levels, 0));
//}
