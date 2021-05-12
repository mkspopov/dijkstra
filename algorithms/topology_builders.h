#pragma once

#include "graph.h"
#include "topology.h"

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

    CompactTopology BuildTopology();

private:
    friend std::pair<Graph, CompactTopology> BuildSimplyTopology(const Graph& graph, LevelId levels);

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

std::pair<Graph, CompactTopology> BuildSimplyTopology(const Graph& graph, LevelId levels);
