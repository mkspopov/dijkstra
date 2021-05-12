#pragma once

#include "multilevel_graph.h"
#include "multilevel_dijkstra.h"

#include <unordered_map>

struct IntermediateGraph {
    IntermediateGraph(Graph graph, CompactTopology topology)
            : builder_(std::move(graph))
            , topology_(std::move(topology))
            , vertices_(topology_.LevelsCount())
    {
        for (VertexId v = 0; v < builder_.graph_.VerticesCount(); ++v) {
            vertices_.at(0).emplace(v, v);
        }
    }

    VertexId GetCellId(VertexId vertex, LevelId level) const {
        assert(vertex < VerticesCount());
        return topology_.GetCellId(vertex, level);
    }

//    VertexId GetCellId(VertexId vertex) const {
//        assert(vertex < VerticesCount());
//        return topology_.GetCellId(vertex);
//    }

    VertexId GetTarget(EdgeId edgeId) const {
        return builder_.graph_.GetTarget(edgeId);
    }

    static inline std::vector<EdgeId> NO_EDGES;
    static inline const auto EMPTY = IteratorRange(NO_EDGES.cbegin(), NO_EDGES.cend());

    auto GetOutgoingEdges(VertexId vertex, LevelId level) const {
        assert(vertex < VerticesCount());
        if (vertices_.at(level).contains(vertex)) {
            return builder_.graph_.GetOutgoingEdges(vertices_.at(level).at(vertex));
        }
        return EMPTY;
    }

    LevelId LevelsCount() const {
        return topology_.LevelsCount();
    }

    VertexId VerticesCount() const {
        return vertices_.at(0).size();
//        return builder_.graph_.VerticesCount();
    }

    auto GetEdgeProperties(EdgeId edgeId) const {
        return builder_.graph_.GetEdgeProperties(edgeId);
    }

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const {
        assert(first < VerticesCount());
        assert(second < VerticesCount());
        return topology_.MaxDistinctLevel(first, second);
    }

private:
    template <class TGraph>
    friend IntermediateGraph SimpleContraction(const TGraph& mlg);

    void AddVertex(VertexId vertex, LevelId level) {
        assert(vertex < VerticesCount());
        if (!vertices_.at(level).contains(vertex)) {
            auto id = builder_.AddVertex();
            vertices_.at(level).emplace(vertex, id);
        }
    }

    void AddEdge(VertexId from, VertexId to, LevelId level, EdgeProperty edgeProperty) {
        assert(from < VerticesCount());
        AddVertex(from, level);
//        assert(to < VerticesCount());
//        AddVertex(to, level);
        builder_.AddEdge(vertices_.at(level).at(from), to, std::move(edgeProperty));
    }

    GraphBuilder builder_;
    CompactTopology topology_;
    std::vector<std::unordered_map<VertexId, VertexId>> vertices_;
};

auto CellInnerTransitions(const IntermediateGraph& graph, VertexId vertex, LevelId level);

struct CellInnerTransitionsS {
    VertexId cell_;
    LevelId level_;
    CellInnerTransitionsS(VertexId cell, LevelId level) : cell_(cell), level_(level) {}

    auto operator()(const IntermediateGraph& graph, VertexId vertex, LevelId level) const {
        auto filter = [this, &graph](EdgeId edgeId) {
            return graph.GetCellId(graph.GetTarget(edgeId), level_) == cell_;
        };
        const auto& range = graph.GetOutgoingEdges(vertex, level);
        return IteratorRange(
            FilterIterator(range.begin(), range.end(), filter),
            FilterIterator(range.end(), range.end(), filter));
    }
};

template <class TMLGraph>
IntermediateGraph SimpleContraction(const TMLGraph& mlg) {
    const auto& zeroLevelGraph = mlg.GetOriginalGraph();
    IntermediateGraph graph(zeroLevelGraph, mlg.GetTopology());

    for (LevelId level = 1; level + 1 < mlg.LevelsCount(); ++level) {
        std::unordered_set<VertexId> border;
        for (const auto& [edgeId, from, to] : zeroLevelGraph.GetEdges()) {
            if (mlg.GetCellId(from, level) != mlg.GetCellId(to, level)) {
                border.insert(from);
                border.insert(to);
                // Adding cut edge:
                graph.AddEdge(from, to, level, zeroLevelGraph.GetEdgeProperties(edgeId));
            }
        }

        for (auto from : border) {
            auto distances = MultilevelDijkstra<StdHeap>(
                graph,
                {from},
                {from},
                CellInnerTransitionsS(mlg.GetCellId(from, level), level));
            for (auto to : border) {
                if (from != to && distances.at(to) < Dijkstra::INF) {
                    graph.AddEdge(from, to, level, {distances.at(to)});
                }
            }
        }
    }
    return graph;
}

//template <>
//IntermediateGraph CliqueContraction(const MultilevelGraph& mlg) {
//    auto zeroLevelGraph = mlg.GetOriginalGraph();
//    IntermediateGraph graph(std::move(zeroLevelGraph), mlg.GetTopology());
//
//    for (LevelId level = 1; level + 1 < mlg.LevelsCount(); ++level) {
//        std::unordered_map<VertexId, std::vector<Edge>> levelCutEdges;
//        std::unordered_map<VertexId, std::vector<VertexId>> borderVertices;
//        for (auto cellId : mlg.GetCells(level)) {
//            auto& border = borderVertices[cellId];
//            auto& cutEdges = levelCutEdges[cellId];
//            // async
//            for (auto u : mlg.GetVertices(cellId)) {
//                for (auto edgeId : mlg.GetOutgoingEdges(u)) {
//                    auto v = mlg.GetTarget(edgeId);
//                    if (cellId != mlg.GetCellId(v, level)) {
//                        cutEdges.emplace_back(edgeId, u, v);
//                        border.push_back(mlg.GetCellId(u, 0));
//                    }
//                }
//            }
//        }
//        // async:WaitAll
//        for (auto&& [cellId, cutEdges] : levelCutEdges) {
//            for (auto [id, u, v] : cutEdges) {
////                graph.AddVertex(u, level);
////                graph.AddVertex(v, level);
//                graph.AddEdge(mlg.GetCellId(u, 0), mlg.GetCellId(v, 0), level, mlg.GetEdgeProperties(id));
////                graph.AddVertex(cellId, level);
////                auto toCell = graph.GetCellId(v, level);
////                graph.AddVertex(toCell);
////                graph.AddEdge(cellId, toCell, mlg.GetEdgeProperties(id));
//            }
//        }
//
//        std::unordered_map<VertexId, std::vector<std::tuple<VertexId, VertexId, Weight>>> levelCellEdges;
//        for (auto cellId : mlg.GetCells(level)) {
//            const auto& border = borderVertices[cellId];
//            auto& cellEdges = levelCellEdges[cellId];
//            // async
//            for (auto u : border) {
//                auto distances = MultilevelDijkstra<StdHeap>(
//                    graph,
//                    {u},
//                    border,
//                    CellInnerTransitions);
//                for (auto v : border) {
//                    if (u != v && distances.at(v) < Dijkstra::INF) {
//                        cellEdges.emplace_back(u, v, distances.at(v));
//                    }
//                }
//            }
//        }
//        // async:WaitAll
//        for (auto&& [cellId, cellEdges] : levelCellEdges) {
//            for (auto [u, v, distance] : cellEdges) {
////                graph.AddVertex(u, level);
////                graph.AddVertex(v, level);
//                graph.AddEdge(u, v, level, EdgeProperty{distance});
//            }
//        }
//    }
//    return graph;
//}
