#pragma once

#include "graph.h"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <ranges>
#include <unordered_set>

struct Topology {
    VertexId GetCellId(VertexId vertexId) const {
        return parents_[vertexId];
    }

    VertexId GetCellId(VertexId vertexId, LevelId level) const {
        auto childOnZeroLevel = vertexId;
        while (childOnZeroLevel >= sizes_.at(1)) {
            childOnZeroLevel = children_[childOnZeroLevel].front();
        }
        return cells_[level][childOnZeroLevel] + sizes_[level];
    }

    LevelId Level(VertexId from) const {
        return std::distance(sizes_.begin(), std::upper_bound(sizes_.begin(), sizes_.end(), from));
    }

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const {
        auto firstLevel = Level(first);
        auto secondLevel = Level(second);

        while (firstLevel < secondLevel) {
            first = parents_[first];
            ++firstLevel;
        }

        while (secondLevel < firstLevel) {
            second = parents_[second];
            ++secondLevel;
        }

        if (first == second) {
            return 0;
        }

        assert(cells_.size() > 1);
        while (parents_[first] != parents_[second]) {
            first = parents_[first];
            second = parents_[second];
            ++firstLevel;
        }
        return firstLevel - 1;
    }

    std::vector<std::vector<VertexId>> cells_;
    std::vector<VertexId> sizes_;

    std::vector<std::vector<VertexId>> cellIds_;
    std::vector<std::vector<VertexId>> children_;
    std::vector<VertexId> parents_;
};

class MultilevelGraph {
public:
    template <class G>
    explicit MultilevelGraph(G&& graph, Topology topology)
        : graph_(std::forward<G>(graph))
        , topology_(std::move(topology))
    {
        assert(topology_.sizes_.front() == 0);
        assert(topology_.sizes_.size() == topology_.cells_.size() + 1);

        VertexId zeroLevelVerticesCount = topology_.cells_.front().size();
        topology_.cellIds_.push_back(topology_.cells_.front());
        topology_.children_.resize(zeroLevelVerticesCount);
        topology_.parents_.resize(zeroLevelVerticesCount);
        GraphBuilder builder(zeroLevelVerticesCount);

        for (LevelId level = 1; level < topology_.cells_.size(); ++level) {
            const auto& layer = topology_.cells_[level];
            topology_.cellIds_.emplace_back();
            std::unordered_set<VertexId> visitedVertices;
            for (auto [zeroLevelId, cellId] : Enumerate(layer)) {
                auto childId = topology_.GetCellId(zeroLevelId, level - 1);
                if (!visitedVertices.contains(cellId)) {
                    visitedVertices.insert(cellId);
                    auto vertexId = builder.AddVertex();
                    topology_.cellIds_.back().push_back(vertexId);
                    topology_.children_.emplace_back();
                    topology_.parents_.emplace_back();
                    if (level > 0) {
                        topology_.children_.back().push_back(childId);
                    }
                } else if (level > 0) {
                    topology_.children_[cellId + topology_.sizes_[level]].push_back(childId);
                }
                topology_.parents_[childId] = cellId + topology_.sizes_[level];
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

    VertexId GetCellId(VertexId vertex) const {
        return topology_.GetCellId(vertex);
    }

    VertexId GetCellId(VertexId vertex, LevelId level) const {
        return topology_.GetCellId(vertex, level);
    }

    auto GetCells(LevelId level) const {
        return topology_.cellIds_[level];
    }

    EdgeProperty GetEdgeProperties(EdgeId edgeId) const {
        return multilevelGraph_.GetEdgeProperties(edgeId);
    }

    const auto& GetOriginalGraph() const {
        return graph_;
    }

    auto GetOutgoingEdges(VertexId from) const {
        return multilevelGraph_.GetOutgoingEdges(from);
    }

    VertexId GetTarget(EdgeId edgeId) const {
        return multilevelGraph_.GetTarget(edgeId);
    }

    const auto& GetVertices(VertexId cellId) const {
        return topology_.children_[cellId];
    }

    LevelId LevelsCount() const {
        return topology_.cells_.size();
    }

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const {
        return topology_.MaxDistinctLevel(first, second);
    }
//
//    auto Vertices(LevelId levelId) const {
//        return Range(topology_.sizes_[levelId], topology_.sizes_[levelId + 1]);
//    }

    VertexId VerticesCount() const {
        return multilevelGraph_.VerticesCount();
    }

    VertexId VerticesCount(LevelId level) const {
        return topology_.sizes_[level + 1] - topology_.sizes_[level];
    }

    const auto& GetTopology() const {
        return topology_;
    }

private:
    Graph graph_;
    Topology topology_;

    Graph multilevelGraph_;
};

// Graph -> (partitioning) -> Topology
// Graph + Topology -> MultilevelGraph
// MultilevelGraph + EdgesWeights -> (contraction) -> RoadGraph

