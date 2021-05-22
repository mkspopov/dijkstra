#pragma once

#include "multilevel_graph.h"

#include <filesystem>
#include <fstream>
#include <unordered_map>

namespace C {
struct Edge {
    VertexId from;
    VertexId to;
    Weight weight;
};

struct LevelStats {
    struct CellStats {
        VertexId count = 0;
        VertexId minSize = std::numeric_limits<VertexId>::max();
        VertexId maxSize = std::numeric_limits<VertexId>::min();

        CellStats& operator+=(const CellStats& rhs) {
            count += rhs.count;
            minSize = std::min(minSize, rhs.minSize);
            maxSize = std::max(maxSize, rhs.maxSize);
            return *this;
        }

        double CalcAverageSize(VertexId verticesCount) const {
            return static_cast<double>(verticesCount) / count;
        }
    };

    LevelStats& operator+=(const LevelStats& rhs) {
        cellStats += rhs.cellStats;
        cutEdges += rhs.cutEdges;
        innerEdges += rhs.innerEdges;
        return *this;
    }

    CellStats cellStats;
    EdgeId cutEdges = 0;
    EdgeId innerEdges = 0;
};
}

std::ostream& operator<<(std::ostream& os, const C::LevelStats::CellStats& stats);

std::ostream& operator<<(std::ostream& os, const C::LevelStats& stats);

struct IntermediateGraph {
    IntermediateGraph() = default;

    IntermediateGraph(WGraph graph, Topology topology);

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

    std::vector<C::LevelStats> stats_;

private:
    friend IntermediateGraph MultithreadContraction(
        const WGraph& originalGraph,
        const Topology& topology);
    friend void TestDumpAndLoad();

    void AddEdge(VertexId from, VertexId to, LevelId level, EdgeProperty edgeProperty);

    void AddVertex(VertexId vertex, LevelId level);

    GraphBuilder<WGraph, EdgeProperty> builder_;
    Topology topology_;
    std::vector<std::unordered_map<VertexId, VertexId>> vertices_;
};

auto CellInnerTransitions(const IntermediateGraph& graph, VertexId vertex, LevelId level);

struct CellInnerTransitionsS {
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

    VertexId cell_;
    LevelId level_;
};

IntermediateGraph MultithreadContraction(
    const WGraph& originalGraph,
    const Topology& topology);
