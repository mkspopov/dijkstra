#pragma once

#include "multilevel_graph.h"
#include "multilevel_dijkstra.h"

#include <unordered_map>

struct IntermediateGraph {
    IntermediateGraph(Graph&& graph, const MultilevelGraph& mlg) : builder_(std::move(graph)), topology_(mlg.GetTopology()) {
        for (VertexId v = 0; v < builder_.graph_.VerticesCount(); ++v) {
            vertices_.insert(v);
        }
    }

    void AddVertex(VertexId vertex) {
        if (!vertices_.contains(vertex)) {
            builder_.AddVertex();
            vertices_.insert(vertex);
        }
    }
    void AddEdge(VertexId from, VertexId to, EdgeProperty edgeProperty) {
        builder_.AddEdge(from, to, std::move(edgeProperty));
    }

    VertexId GetCellId(VertexId vertex, LevelId level) const {
        return topology_.GetCellId(vertex, level);
    }

    VertexId GetCellId(VertexId vertex) const {
        return topology_.GetCellId(vertex);
    }

    VertexId GetTarget(EdgeId edgeId) const {
        return builder_.graph_.GetTarget(edgeId);
    }

    auto GetOutgoingEdges(VertexId vertexId) const {
        return builder_.graph_.GetOutgoingEdges(vertexId);
    }

    LevelId LevelsCount() const {
        return topology_.cells_.size();
    }

    VertexId VerticesCount() const {
        return builder_.graph_.VerticesCount();
    }

    auto GetEdgeProperties(EdgeId edgeId) const {
        return builder_.graph_.GetEdgeProperties(edgeId);
    }

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const {
        return topology_.MaxDistinctLevel(first, second);
    }

    const Topology& topology_;
    std::unordered_set<VertexId> vertices_;

    GraphBuilder builder_;
};

auto CellInnerTransitions(const IntermediateGraph& graph, VertexId vertexId) {
    auto filter = [&graph, cell = graph.GetCellId(vertexId)](EdgeId edgeId) {
        return graph.GetCellId(graph.GetTarget(edgeId)) == cell;
    };
    const auto& range = graph.GetOutgoingEdges(vertexId);
    return IteratorRange(
        FilterIterator(range.begin(), range.end(), filter),
        FilterIterator(range.end(), range.end(), filter));
}

IntermediateGraph CliqueContraction(const MultilevelGraph& mlg) {
    auto zeroLevelGraph = mlg.GetOriginalGraph();
    IntermediateGraph graph(std::move(zeroLevelGraph), mlg);

    for (LevelId level = 1; level + 1 < mlg.LevelsCount(); ++level) {
        std::unordered_map<VertexId, std::vector<Edge>> levelCutEdges;
        std::unordered_map<VertexId, std::vector<VertexId>> borderVertices;
        for (auto cellId : mlg.GetCells(level)) {
            auto& border = borderVertices[cellId];
            auto& cutEdges = levelCutEdges[cellId];
            // async
            for (auto u : mlg.GetVertices(cellId)) {
                for (auto edgeId : mlg.GetOutgoingEdges(u)) {
                    auto v = mlg.GetTarget(edgeId);
                    if (cellId != mlg.GetCellId(v, level)) {
                        cutEdges.emplace_back(edgeId, u, v);
                        border.push_back(u);
                    }
                }
            }
        }
        // async:WaitAll
        for (auto&& [cellId, cutEdges] : levelCutEdges) {
            for (auto [id, u, v] : cutEdges) {
                graph.AddVertex(u);
                graph.AddVertex(v);
                graph.AddEdge(u, v, mlg.GetEdgeProperties(id));
                graph.AddVertex(cellId);
                auto toCell = graph.GetCellId(v, level);
                graph.AddVertex(toCell);
                graph.AddEdge(cellId, toCell, mlg.GetEdgeProperties(id));
            }
        }

        std::unordered_map<VertexId, std::vector<std::tuple<VertexId, VertexId, Weight>>> levelCellEdges;
        for (auto cellId : mlg.GetCells(level)) {
            const auto& border = borderVertices[cellId];
            auto& cellEdges = levelCellEdges[cellId];
            // async
            for (auto u : border) {
                auto distances = MultiLevelDijkstra<StdHeap>(
                    graph,
                    {u},
                    border,
                    CellInnerTransitions);
                for (auto v : border) {
                    if (u != v && distances[v] < Dijkstra::INF) {
                        cellEdges.emplace_back(u, v, distances[v]);
                    }
                }
            }
        }
        // async:WaitAll
        for (auto&& [cellId, cellEdges] : levelCellEdges) {
            for (auto [u, v, distance] : cellEdges) {
                graph.AddVertex(u);
                graph.AddVertex(v);
                graph.AddEdge(u, v, EdgeProperty{distance});
            }
        }
    }
    return graph;
}