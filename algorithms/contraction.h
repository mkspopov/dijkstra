#pragma once

#include "multilevel_graph.h"

#include <filesystem>
#include <fstream>
#include <unordered_map>

struct IntermediateGraph {
    IntermediateGraph() = default;

    IntermediateGraph(Graph graph, CompactTopology topology);

    void Dump(std::ostream& out) const;

    VertexId GetCellId(VertexId vertex, LevelId level) const;

    EdgeProperty GetEdgeProperties(EdgeId edgeId) const;

    auto GetOutgoingEdges(VertexId vertex, LevelId level) const {
        static const std::vector<EdgeId> NO_EDGES;
        static const auto EMPTY = IteratorRange(NO_EDGES.cbegin(), NO_EDGES.cend());

        ASSERT(vertex < VerticesCount());
        if (vertices_.at(level).contains(vertex)) {
            return builder_.graph_.GetOutgoingEdges(vertices_.at(level).at(vertex));
        }
        return EMPTY;
    }


    VertexId GetTarget(EdgeId edgeId) const;

    LevelId LevelsCount() const;

    void Load(std::istream& in);

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const;

    VertexId VerticesCount() const;

private:
    friend IntermediateGraph SimpleContraction(
        const Graph& originalGraph,
        const CompactTopology& topology);

    void AddEdge(VertexId from, VertexId to, LevelId level, EdgeProperty edgeProperty);

    void AddVertex(VertexId vertex, LevelId level);

    GraphBuilder builder_;
    CompactTopology topology_;
    std::vector<std::unordered_map<VertexId, VertexId>> vertices_;
};

auto CellInnerTransitions(const IntermediateGraph& graph, VertexId vertex, LevelId level);

struct CellInnerTransitionsS {
    VertexId cell_;
    LevelId level_;
    CellInnerTransitionsS(VertexId cell, LevelId level);

    auto operator()(
        const IntermediateGraph& graph,
        VertexId vertex,
        LevelId level) const
    {
        auto filter = [this, &graph](EdgeId edgeId) {
            return graph.GetCellId(graph.GetTarget(edgeId), level_) == cell_;
        };
        const auto& range = graph.GetOutgoingEdges(vertex, level);
        return IteratorRange(
            FilterIterator(range.begin(), range.end(), filter),
            FilterIterator(range.end(), range.end(), filter));
    }
};

IntermediateGraph SimpleContraction(
    const Graph& originalGraph,
    const CompactTopology& topology);

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
