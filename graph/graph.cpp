//
// Created by mkspopov on 11.11.2020.
//

#include "graph.h"

#include <cassert>

EdgeProperty Graph::GetEdgeProperties(EdgeId edgeId) const {
    return edgeProperties_[edgeId];
}

IteratorRange<std::vector<EdgeId>::const_iterator> Graph::GetOutgoingEdges(VertexId from) const {
    return IteratorRange(
            adjacencyList_[from].begin(),
            adjacencyList_[from].end());
}

VertexId Graph::GetTarget(EdgeId edgeId) const {
    return edges_[edgeId].to;
}

VertexId GraphBuilder::AddVertex() {
    graph_.adjacencyList_.emplace_back();
    return graph_.adjacencyList_.size() - 1;
}

EdgeId GraphBuilder::AddEdge(VertexId from, VertexId to, EdgeProperty properties) {

    for (auto edgeId : graph_.GetOutgoingEdges(from)) {
        if (graph_.GetTarget(edgeId) == to && graph_.GetEdgeProperties(edgeId) == properties) {
            return UNDEFINED;
        }
    }

    const EdgeId edgeId = graph_.edges_.size();
    graph_.edges_.emplace_back(to);
    graph_.edgeProperties_.emplace_back(std::move(properties));
    graph_.adjacencyList_[from].push_back(edgeId);
    return edgeId;
}

VertexId Graph::VerticesCount() const {
    return adjacencyList_.size();
}

EdgeId Graph::EdgesCount() const {
    return edges_.size();
}

Graph&& GraphBuilder::Build() {
    assert(!built_);
    built_ = true;
    return std::move(graph_);
}

GraphBuilder::GraphBuilder(VertexId verticesCount) {
    graph_.adjacencyList_.resize(verticesCount);
}

Edge::Edge(VertexId to)
    : to(to) {
}

Graph Graph::Reversed() const {
    GraphBuilder builder;
    for (size_t i = 0; i < adjacencyList_.size(); ++i) {
        builder.AddVertex();
    }
    for (size_t from = 0; from < adjacencyList_.size(); ++from) {
        for (const auto edgeId : GetOutgoingEdges(from)) {
            builder.AddEdge(GetTarget(edgeId), from, edgeProperties_[edgeId]);
        }
    }
    return builder.Build();
}
