#include "contraction.h"
#include "topology_builders.h"

#include <execution>
#include <iostream>
#include <vector>

struct State {

};

struct TopGraph {
    TopGraph(Graph graph, LevelId levels)
        : builder_(std::move(graph))
        , parents_(levels)
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

    auto BuildTopology() {
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
    friend CompactTopology BuildSimplyTopology(const Graph& graph, LevelId levels);

    VertexId AddVertex(VertexId vertex, LevelId level) {
        auto id = builder_.AddVertex();
        if (layers_.size() <= level) {
            layers_.emplace_back();
        }
        layers_.at(level).push_back(id);
        SetCell(vertex, id);
        return id;
    }

    void AddEdge(VertexId from, VertexId to, LevelId level, EdgeProperty edgeProperty) {
        builder_.AddEdge(from, to, std::move(edgeProperty));
    }

    void SetCell(VertexId vertexId, VertexId cellId) {
        auto [_, emplaced] = parents_.emplace(vertexId, cellId);
        ASSERT(emplaced);
    }

    GraphBuilder builder_;
    std::vector<std::vector<VertexId>> layers_;
    std::unordered_map<VertexId, VertexId> parents_;
};

CompactTopology BuildSimplyTopology(const Graph& graph, LevelId levels) {
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
                    topGraph.AddEdge(topGraph.GetCellId(from), topGraph.GetCellId(to), level, topGraph.GetEdgeProperties(edgeId));
                }
            }
        }
    }

    return topGraph.BuildTopology();
}
