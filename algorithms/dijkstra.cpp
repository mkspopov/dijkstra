//
// Created by mkspopov on 11.11.2020.
//

#include "dijkstra.h"

Dijkstra::Dijkstra(const Graph& graph)
    : graph_(graph)
{
}

bool Dijkstra::IsAffected(VertexId vertexId) const {
    return distances_[vertexId] != INF;
}

bool Dijkstra::IsProcessed(VertexId vertexId) const {
    return colors_[vertexId] == Color::BLACK;
}

void Dijkstra::Preprocess() {
    distances_.resize(graph_.VerticesCount(), INF);
    colors_.resize(graph_.VerticesCount(), Color::WHITE);
    affectedVertices_.reserve(graph_.VerticesCount());
}

Weight Dijkstra::FindShortestPathWeight(VertexId source, VertexId target) {
    InitSearch(source);
    while (!heap_.Empty()) {
        const auto [from, fromDistance] = heap_.Extract();
        if (from == target) {
            return fromDistance;
        }
        ProcessVertex(from, fromDistance);
    }
    return distances_[target];
}

void Dijkstra::FindShortestPathsWeights(VertexId source) {
    InitSearch(source);
    while (!heap_.Empty()) {
        const auto [from, fromDistance] = heap_.Extract();
        ProcessVertex(from, fromDistance);
    }
}

VertexId Dijkstra::ProcessVertex() {
    if (heap_.Empty()) {
        return UNDEFINED_VERTEX;
    }
    const auto [from, fromDistance] = heap_.Extract();
    ProcessVertex(from, fromDistance);
    return from;
}

void Dijkstra::ProcessVertex(VertexId from, float fromDistance) {
    ExamineVertex(from);

    if (fromDistance > distances_[from]) {
        return;
    }
    if (colors_[from] == Color::BLACK) {
        return;
    }
    ASSERT_EQUAL(fromDistance, distances_[from]);

    for (const auto edgeId : graph_.GetOutgoingEdges(from)) {
        ExamineEdge(edgeId);

        const auto to = graph_.GetTarget(edgeId);
        const auto edgeWeight = graph_.GetEdgeProperties(edgeId).weight;
        const auto distance = fromDistance + edgeWeight;

        if (distances_[to] > distance) {
            distances_[to] = distance;
            if (colors_[to] == Color::WHITE) {
                DiscoverVertex(to);
                heap_.Emplace(to, distance);
                colors_[to] = Color::GRAY;
                RelaxEdge(edgeId);
            } else {
                ASSERT(colors_[to] == Color::GRAY);
                heap_.Decrease({to, distance});
                RelaxEdge(edgeId);
            }
        } else {
            NotRelaxed(edgeId);
        }
    }

    colors_[from] = Color::BLACK;
}

Weight Dijkstra::GetShortestDistance(VertexId target) const {
    return distances_[target];
}

void Dijkstra::InitSearch(VertexId source) {
    Clear();

    DiscoverVertex(source);
    distances_[source] = START_WEIGHT;
    heap_.Emplace(source, START_WEIGHT);
    colors_[source] = Color::GRAY;
}

void Dijkstra::NotRelaxed(EdgeId) {
}

void Dijkstra::RelaxEdge(EdgeId) {
}

void Dijkstra::ExamineEdge(EdgeId) {
}

void Dijkstra::ExamineVertex(VertexId vertex) {
    const auto erasedCount = targets_.erase(vertex);
    if (erasedCount > 0 && targets_.empty()) {
        ClearHeap();
    }
}

void Dijkstra::DiscoverVertex(VertexId vertex) {
    affectedVertices_.push_back(vertex);
}

void Dijkstra::Clear() {
    for (const auto vertex : affectedVertices_) {
        distances_[vertex] = INF;
        colors_[vertex] = Color::WHITE;
    }
    affectedVertices_.clear();
    ClearHeap();
}

void Dijkstra::ClearHeap() {
    while (!heap_.Empty()) {
        heap_.Extract();
    }
}
