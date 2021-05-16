#include "contraction.h"
#include "multilevel_dijkstra.h"

#include <iostream>
#include <vector>

IntermediateGraph::IntermediateGraph(WeightGraph<EdgeProperty> graph, CompactTopology topology)
    : builder_(std::move(graph))
    , topology_(std::move(topology))
    , vertices_(topology_.LevelsCount())
{
    for (VertexId v = 0; v < builder_.graph_.VerticesCount(); ++v) {
        vertices_.at(0).emplace(v, v);
    }
}

void IntermediateGraph::AddEdge(VertexId from, VertexId to, LevelId level, EdgeProperty edgeProperty) {
    ASSERT(from < VerticesCount());
    AddVertex(from, level);
    ASSERT(to < VerticesCount());
    builder_.AddEdge(vertices_.at(level).at(from), to, std::move(edgeProperty));
}

void IntermediateGraph::AddVertex(VertexId vertex, LevelId level) {
    ASSERT(vertex < VerticesCount());
    if (!vertices_.at(level).contains(vertex)) {
        auto id = builder_.AddVertex();
        vertices_.at(level).emplace(vertex, id);
    }
}

void IntermediateGraph::Dump(std::ostream& os) const {
    builder_.graph_.Dump(os);
    topology_.Dump(os);
    ::Dump(os, vertices_);
}

VertexId IntermediateGraph::GetCellId(VertexId vertex, LevelId level) const {
    ASSERT(vertex < VerticesCount());
    return topology_.GetCellId(vertex, level);
}

EdgeProperty IntermediateGraph::GetEdgeProperties(EdgeId edgeId) const {
    return builder_.graph_.GetEdgeProperties(edgeId);
}

VertexId IntermediateGraph::GetTarget(EdgeId edgeId) const {
    return builder_.graph_.GetTarget(edgeId);
}

LevelId IntermediateGraph::LevelsCount() const {
    return topology_.LevelsCount() - 1;
}

void IntermediateGraph::Load(std::istream& is) {
    builder_.graph_.Load(is);
    topology_.Load(is);
    ::Load(is, vertices_);
}

LevelId IntermediateGraph::MaxDistinctLevel(VertexId first, VertexId second) const {
    ASSERT(first < VerticesCount());
    ASSERT(second < VerticesCount());
    return topology_.MaxDistinctLevel(first, second);
}

VertexId IntermediateGraph::VerticesCount() const {
    return vertices_.at(0).size();
}

auto CellInnerTransitions(const IntermediateGraph& graph, VertexId vertex, LevelId level) {
    auto cell = graph.GetCellId(vertex, level);
    auto filter = [=, &graph](EdgeId edgeId) {
        return graph.GetCellId(graph.GetTarget(edgeId), level) == cell;
    };
    const auto& range = graph.GetOutgoingEdges(vertex, level);
    return IteratorRange(
        FilterIterator(range.begin(), range.end(), filter),
        FilterIterator(range.end(), range.end(), filter));
}

CellInnerTransitionsS::CellInnerTransitionsS(VertexId cell, LevelId level)
    : cell_(cell)
    , level_(level)
{}

template <class Function>
void TraverseChildren(
    VertexId vertex,
    const std::unordered_map<VertexId, std::vector<VertexId>>& children,
    const Function& function)
{
    if (!children.contains(vertex)) {
        function(vertex);
        return;
    }
    for (auto child : children.at(vertex)) {
        TraverseChildren(child, children, function);
    }
}

IntermediateGraph SimpleContraction(const WeightGraph<EdgeProperty>& originalGraph, const CompactTopology& topology) {
    IntermediateGraph graph(originalGraph, topology);

    ASSERT(topology.LevelsCount() > 1);
    const auto lastLevel = topology.LevelsCount() - 1;
//    std::unordered_map<VertexId, std::vector<VertexId>> children;
//    for (const auto [child, parent] : Enumerate(topology.parents_)) {
//        children[parent].push_back(child);
//    }

    for (LevelId level = 1; level <= lastLevel; ++level) {
        Log() << "Contracting level" << static_cast<int>(level) << "...";
        Timer timer;
        std::unordered_map<VertexId, std::unordered_set<VertexId>> borders;
        for (const auto& [edgeId, from, to] : originalGraph.GetEdges()) {
            if (topology.GetCellId(from, level) != topology.GetCellId(to, level)) {
                borders[topology.GetCellId(from, level)].insert(from);
                borders[topology.GetCellId(to, level)].insert(to);
                // Adding cut edge:
                graph.AddEdge(from, to, level, originalGraph.GetEdgeProperties(edgeId));
            }
        }

        for (const auto& [_, border] : borders) {
            for (auto from : border) {
                auto distances = MultilevelDijkstra<StdHeap>(
                    graph,
                    {from},
                    {from},
                    CellInnerTransitionsS(topology.GetCellId(from, level), level),
                    GetTrivialVisitor());
                for (auto to : border) {
                    if (from != to && distances.at(to) < Dijkstra::INF) {
                        graph.AddEdge(from, to, level, {distances.at(to)});
                    }
                }
            }
        }
        Log() << "level" << static_cast<int>(level) << "contracted in"
              << timer.Elapsed() / 1'000'000 << "ms";
        Log() << "vertices:" << graph.vertices_.at(level).size();
    }

    return graph;
}
