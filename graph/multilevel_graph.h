#pragma once

#include "graph.h"
#include "topology.h"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <ranges>
#include <unordered_set>

class MultilevelGraph {
public:
    template <class G>
    MultilevelGraph(G&& graph, Topology topology)
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
            const auto& layer = topology_.cells_.at(level);
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
                    topology_.children_.at(cellId + topology_.sizes_[level]).push_back(childId);
                }
                topology_.parents_.at(childId) = cellId + topology_.sizes_.at(level);
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
        return topology_.cellIds_.at(level);
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
        return topology_.children_.at(cellId);
    }

    LevelId LevelsCount() const {
        return topology_.cells_.size();
    }

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const {
        return topology_.MaxDistinctLevel(first, second);
    }
//
//    auto Vertices(LevelId levelId) const {
//        return Range(topology_.sizes_.at(levelId), topology_.sizes_.at(levelId + 1));
//    }

    VertexId VerticesCount() const {
        return multilevelGraph_.VerticesCount();
    }

    VertexId VerticesCount(LevelId level) const {
        return topology_.sizes_.at(level + 1) - topology_.sizes_.at(level);
    }

    const auto& GetTopology() const {
        return topology_;
    }

private:
    Graph graph_;
    Topology topology_;

    Graph multilevelGraph_;
};

class CompactMultilevelGraph {
public:
    template <class G, class T>
    explicit CompactMultilevelGraph(G&& graph, T&& topology)
        : graph_(std::forward<G>(graph))
        , topology_(std::forward<T>(topology))
    {
        assert(topology_.sizes_.front() == 0);
    }

    VertexId GetCellId(VertexId vertex) const {
        return topology_.GetCellId(vertex);
    }

    VertexId GetCellId(VertexId vertex, LevelId level) const {
        return topology_.GetCellId(vertex, level);
    }

//    auto GetCells(LevelId level) const {
//        return topology_.cellIds_.at(level);
//    }

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

    LevelId LevelsCount() const {
        return topology_.sizes_.size();
    }

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const {
        return topology_.MaxDistinctLevel(first, second);
    }

    VertexId VerticesCount() const {
        return multilevelGraph_.VerticesCount();
    }

    VertexId VerticesCount(LevelId level) const {
        return topology_.sizes_.at(level + 1) - topology_.sizes_.at(level);
    }

    const auto& GetTopology() const {
        return topology_;
    }

private:
    Graph graph_;
    CompactTopology topology_;

    Graph multilevelGraph_;
};
