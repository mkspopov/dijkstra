//
// Created by mkspopov on 11.11.2020.
//

#include "dijkstra.h"

Dijkstra::Dijkstra(const WGraph& graph)
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
    const auto erasedCount = targets_.erase(from);
    if (erasedCount > 0 && targets_.empty()) {
        ClearHeap();
        return;
    }

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
                EdgeRelaxed(edgeId);
            } else {
                ASSERT(colors_[to] == Color::GRAY);
                heap_.Decrease({to, distance});
                EdgeRelaxed(edgeId);
            }
        } else {
            EdgeNotRelaxed(edgeId);
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

void Dijkstra::DiscoverVertex(VertexId vertex) {
    affectedVertices_.push_back(vertex);
    ++stats_.verticesDiscovered;
}

void Dijkstra::ExamineEdge(EdgeId) {
    ++stats_.edgesExamined;
}

void Dijkstra::ExamineVertex(VertexId) {
    ++stats_.verticesExamined;
}

void Dijkstra::EdgeNotRelaxed(EdgeId) {
    ++stats_.edgesNotRelaxed;
}

void Dijkstra::EdgeRelaxed(EdgeId) {
    ++stats_.edgesRelaxed;
}

void Dijkstra::FinishVertex(VertexId) {
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

std::ostream& operator<<(std::ostream& os, const Dijkstra::Stats& s) {
    os << "verticesDiscovered " << s.verticesDiscovered << '\n';
    os << "verticesExamined " << s.verticesExamined << '\n';
    os << "edgesExamined " << s.edgesExamined << '\n';
    os << "edgesRelaxed " << s.edgesRelaxed << '\n';
    os << "edgesNotRelaxed " << s.edgesNotRelaxed << '\n';
    return os;
}

Dijkstra::Stats& operator+=(Dijkstra::Stats& lhs, const Dijkstra::Stats& rhs) {
    lhs.verticesDiscovered += rhs.verticesDiscovered;
    lhs.verticesExamined += rhs.verticesExamined;
    lhs.edgesExamined += rhs.edgesExamined;
    lhs.edgesRelaxed += rhs.edgesRelaxed;
    lhs.edgesNotRelaxed += rhs.edgesNotRelaxed;
    return lhs;
}

Dijkstra::Stats operator/(Dijkstra::Stats lhs, int rhs) {
    lhs.verticesDiscovered /= rhs;
    lhs.verticesExamined /= rhs;
    lhs.edgesExamined /= rhs;
    lhs.edgesRelaxed /= rhs;
    lhs.edgesNotRelaxed /= rhs;
    return lhs;
}
