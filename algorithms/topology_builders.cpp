#include "contraction.h"
#include "graph_traverse.h"
#include "inertial_flow.h"
#include "topology_builders.h"

#include <execution>
#include <iostream>
#include <vector>

struct TopGraph {
    TopGraph(WGraph graph, LevelId levels)
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
    friend CompactTopology BuildSimplyTopology(const WGraph& graph, LevelId levels);

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

    GraphBuilder<WGraph, EdgeProperty> builder_;
    std::vector<std::vector<VertexId>> layers_;
    std::unordered_map<VertexId, VertexId> parents_;
};

CompactTopology BuildSimplyTopology(const WGraph& graph, LevelId levels) {
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

struct XCoordComp {
    XCoordComp(const CoordGraph& graph) : graph_(graph) {
    }

    bool operator()(VertexId lhs, VertexId rhs) const {
        return graph_.GetVertexProperties(lhs).x < graph_.GetVertexProperties(rhs).x;
    }

private:
    const CoordGraph& graph_;
};

CompactTopology BuildTopologyInertialFlow(const CoordGraph& graph, LevelId levels, double coef, int steps) {
    auto states = MakePartition<XCoordComp>(graph, levels, coef, steps);

    /*
     * 0 0 1 1 1 0  ->  6 6 7  7  7  6
     * 0 1 2 3 3 1  ->  8 9 10 11 11 9
     */

    CompactTopology topology;
    topology.sizes_ = {0, graph.VerticesCount()};
    std::vector<std::vector<VertexId>> cells(levels, std::vector<VertexId>(graph.VerticesCount()));
    for (LevelId level : Range(0ul, states.size())) {
        size_t index = states.size() - level - 1;
        std::unordered_set<VertexId> newCells;
        for (auto [vertex, cellId] : Enumerate(states.at(index).toGraphId)) {
            auto parentId = cellId + topology.sizes_.back();
            cells.at(level).at(vertex) = parentId;
            newCells.insert(parentId);
            auto child = vertex;
            if (level > 0) {
                child = cells.at(level - 1).at(vertex);
            }
            while (topology.parents_.size() <= child) {
                topology.parents_.emplace_back();
            }
            topology.parents_.at(child) = parentId;
        }
        topology.sizes_.push_back(newCells.size() + topology.sizes_.back());
    }

    auto root = topology.sizes_.back();
    topology.sizes_.push_back(root + 1);
    for (auto child : cells.at(levels - 1)) {
        while (topology.parents_.size() <= child) {
            topology.parents_.emplace_back();
        }
        topology.parents_.at(child) = root;
    }

    return topology;
}
