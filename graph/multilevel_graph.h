#pragma once

#include "graph.h"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <ranges>
#include <unordered_set>

struct Topology {
    VertexId GetCellId(VertexId vertexId, LevelId level) const {
        auto nextLevel = std::upper_bound(sizes_.begin(), sizes_.end(), vertexId);
        auto lowerLevelsVerticesCount = *(--nextLevel);
        assert(lowerLevelsVerticesCount <= vertexId);
        auto childOnZeroLevel = vertexId - lowerLevelsVerticesCount;
        return cells_[level][childOnZeroLevel] + sizes_[level];
    }

    VertexId GetCellId(VertexId vertexId) const {
        return GetCellId(vertexId, Level(vertexId));
    }

    LevelId Level(VertexId from) const {
        return std::distance(sizes_.begin(), std::upper_bound(sizes_.begin(), sizes_.end(), from));
    }

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const {
        assert(cells_.size() > 1);
        LevelId level = 1;
        while (cells_[level][first] != cells_[level][second]) {
            ++level;
        }
        return level - 1;
    }

    std::vector<std::vector<VertexId>> cells_;
    std::vector<VertexId> sizes_;

    std::vector<std::vector<VertexId>> cellIds_;
    std::vector<std::vector<VertexId>> children_;
};

class MultilevelGraph {
public:
    explicit MultilevelGraph(const Graph& graph, Topology topology)
        : graph_(graph)
        , topology_(std::move(topology))
    {
        assert(topology_.sizes_.front() == 0);
        assert(topology_.sizes_.size() == topology_.cells_.size() + 1);

        GraphBuilder builder;
        for (const auto& [level, layer] : Enumerate(topology_.cells_)) {
            topology_.cellIds_.emplace_back();
            std::unordered_set<VertexId> visitedVertices;
            for (auto [id, cellId] : Enumerate(layer)) {
                if (!visitedVertices.contains(cellId)) {
                    visitedVertices.insert(cellId);
                    auto vertexId = builder.AddVertex();
                    topology_.cellIds_.back().push_back(vertexId);
                    topology_.children_.emplace_back();
                    if (level > 0) {
                        topology_.children_.back().push_back(GetCellId(id, level - 1));
                    }
                } else if (level > 0) {
                    topology_.children_[cellId + topology_.sizes_[level]].push_back(GetCellId(id, level - 1));
                }
            }
        }

        for (auto& children : topology_.children_) {
            std::ranges::sort(children);
            auto [first, last] = std::ranges::unique(children);
            children.erase(first, last);
        }

        // for each vertex we have component edges and cut edges
        for (LevelId level = 0; level < LevelsCount(); ++level) {
            for (const auto& [edgeId, edge] : Enumerate(graph_.GetEdges())) {
                auto cellFrom = GetCellId(edge.from, level);
                auto cellTo = GetCellId(edge.to, level);
                if (cellFrom != cellTo) {
                    builder.AddEdge(cellFrom, cellTo, graph_.GetEdgeProperties(edgeId));
                }
            }
        }

        multilevelGraph_ = builder.Build();
    }

    EdgeProperty GetEdgeProperties(EdgeId edgeId) const;

    auto GetOutgoingEdges(VertexId from) const {
        return multilevelGraph_.GetOutgoingEdges(from);
    }

    VertexId GetTarget(EdgeId edgeId) const {
        return graph_.GetTarget(edgeId);
    }

    VertexId VerticesCount() const {
        return graph_.VerticesCount();
    }

    VertexId VerticesCount(LevelId level) const {
        if (level == 0) {
            return VerticesCount();
        }
        return topology_.sizes_[level + 1] - topology_.sizes_[level];
    }

//    EdgeId EdgesCount() const {
//        return edges_.size();
//    }

    Graph Reversed() const;

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const {
        return topology_.MaxDistinctLevel(first, second);
    }

    VertexId GetCellId(VertexId vertex) const {
        return topology_.GetCellId(vertex);
    }

    VertexId GetCellId(VertexId vertex, LevelId level) const {
        return topology_.GetCellId(vertex, level);
    }

    LevelId LevelsCount() const {
        return topology_.cells_.size();
    }

    auto Vertices(LevelId levelId) const {
        return Range(topology_.sizes_[levelId], topology_.sizes_[levelId + 1]);
    }

    const std::vector<Edge>&
    GetEdges() const {
        return multilevelGraph_.GetEdges();
    }

    const std::vector<Edge>&
    GetEdges0() const {
        return graph_.GetEdges();
    }

    const auto& GetGraph() const {
        return graph_;
    }

    auto GetCells(LevelId level) const {
        return topology_.cellIds_[level] | std::views::transform([&](auto cellId) {
            return cellId + topology_.sizes_[level];
        });
    }

    const auto& GetVertices(VertexId cellId) const {
        return topology_.children_[cellId];
    }

private:
    const Graph& graph_;
    Topology topology_;

    Graph multilevelGraph_;
};

// Graph -> (partitioning) -> Topology
// Graph + Topology -> MultilevelGraph
// MultilevelGraph + EdgesWeights -> (contraction) -> RoadGraph

