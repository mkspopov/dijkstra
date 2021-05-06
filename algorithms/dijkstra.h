//
// Created by mkspopov on 11.11.2020.
//

#pragma once

#include "heap.h"
#include "graph.h"
#include "visitor.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <unordered_set>
#include <vector>

class Dijkstra {
public:
    static inline constexpr Weight INF = std::numeric_limits<Weight>::infinity();
    static inline constexpr Weight START_WEIGHT = 0;
    static inline constexpr VertexId UNDEFINED_VERTEX = std::numeric_limits<VertexId>::max();

    explicit Dijkstra(const Graph& graph)
        : graph_(graph)
    {
    }

    auto AffectedVertices() {
        return IteratorRange(affectedVertices_.begin(), affectedVertices_.end());
    }

    bool IsAffected(VertexId vertexId) const {
        return distances_[vertexId] != INF;
    }

    bool IsProcessed(VertexId vertexId) const {
        return isVertexProcessed_[vertexId];
    }

    void Preprocess() {
        distances_.resize(graph_.VerticesCount(), INF);
        isVertexProcessed_.resize(graph_.VerticesCount(), false);
        affectedVertices_.reserve(graph_.VerticesCount());
    }

    Weight FindShortestPathWeight(VertexId source, VertexId target) {
        InitSearch(source);
        while (!heap_.empty()) {
            const auto [from, fromDistance] = heap_.top();
            if (from == target) {
                return fromDistance;
            }
            ProcessVertex(from, fromDistance);
        }
        return distances_[target];
    }

    void FindShortestPathsWeights(VertexId source) {
        InitSearch(source);
        while (!heap_.empty()) {
            const auto [from, fromDistance] = heap_.top();
            ProcessVertex(from, fromDistance);
        }
    }

    VertexId ProcessVertex() {
        if (heap_.empty()) {
            return UNDEFINED_VERTEX;
        }
        const auto [from, fromDistance] = heap_.top();
        ProcessVertex(from, fromDistance);
        return from;
    }

    void ProcessVertex(int from, float fromDistance) {
        heap_.pop();
        ExamineVertex(from);

        if (fromDistance > distances_[from]) {
            return;
        }
        assert(fromDistance == distances_[from]);

        for (const auto edgeId : graph_.GetOutgoingEdges(from)) {
            ExamineEdge(edgeId);

            const auto to = graph_.GetTarget(edgeId);
            const auto edgeWeight = graph_.GetEdgeProperties(edgeId).weight;
            const auto distance = fromDistance + edgeWeight;

            if (!IsVisited(to)) {
                DiscoverVertex(to);
                distances_[to] = distance;
                heap_.emplace(to, distance);
            } else if (distances_[to] > distance) {
                distances_[to] = distance;
                heap_.emplace(to, distance);
                RelaxEdge(edgeId);
            } else {
                NotRelaxed(edgeId);
            }
        }

        isVertexProcessed_[from] = true;
    }

    /*
     * Set targets to prevent Dijkstra from unnecessary running.
     */
    template <class Container>
    void SetTargets(const Container& targets) {
        assert(targets_.empty());
        for (const auto target : targets) {
            targets_.insert(target);
        }
    }

    /*
     * Get the shortest distance after FindShortestPathsWeights.
     */
    Weight GetShortestDistance(VertexId target) const {
        return distances_[target];
    }

    void InitSearch(VertexId source) {
        Clear();

        DiscoverVertex(source);
        distances_[source] = START_WEIGHT;
        heap_.emplace(source, START_WEIGHT);
    }

private:
    void NotRelaxed(EdgeId ) {
    }

    void RelaxEdge(EdgeId ) {
    }

    void ExamineEdge(EdgeId ) {
    }

    void ExamineVertex(VertexId vertex) {
        const auto erasedCount = targets_.erase(vertex);
        if (erasedCount > 0 && targets_.empty()) {
            ClearHeap();
        }
    }

    bool IsVisited(VertexId vertex) const {
        return distances_[vertex] != INF;
    }

    void DiscoverVertex(VertexId vertex) {
        affectedVertices_.push_back(vertex);
    }

    void Clear() {
        for (const auto vertex : affectedVertices_) {
            distances_[vertex] = INF;
            isVertexProcessed_[vertex] = false;
        }
        affectedVertices_.clear();
        ClearHeap();
    }

    void ClearHeap() {
        heap_ = Heap<HeapElement>();
    }

    const Graph& graph_;
    std::vector<Weight> distances_;
    std::vector<VertexId> affectedVertices_;
    std::vector<char> isVertexProcessed_;  // char is faster than bool.
    Heap<HeapElement> heap_;

    std::unordered_set<VertexId> targets_;
};

enum class Color {
    WHITE,
    GRAY,
    BLACK,
};

template <class Queue, class Visitor>
void BoostDijkstra(
    const Graph& graph,
    std::vector<Weight>& distances,
    std::vector<Color>& colors,
    const std::vector<VertexId>& sources,
    Queue& queue,
    Visitor& visitor)
{
    for (auto source : sources) {
        distances[source] = 0;
        colors[source] = Color::GRAY;
        queue.Push(source, 0);
        visitor.SourceVertex(source);
    }
    
    while (!queue.Empty()) {
        auto [from, dist] = queue.Extract();
        // TODO: Optional check to easily deal with 
        // std::priority_queue. This may slow down
        // code because of memory load of a random vertex
        // of the graph. So, measure time without it.
        if (colors[from] == Color::BLACK) {
            continue;
        }
        visitor.ExamineVertex(from);

        for (auto edgeId : graph.GetOutgoingEdges(from)) {
            visitor.ExamineEdge(edgeId);
            auto to = graph.GetTarget(edgeId);
            auto relaxedDist = dist + graph.GetEdgeProperties(edgeId).weight;
            if (relaxedDist < distances[to]) {
                distances[to] = relaxedDist;
                colors[to] = Color::GRAY;
                if (colors[to] == Color::WHITE) {
                    queue.Push(to, relaxedDist);
                } else if (colors[to] == Color::GRAY) {
                    queue.Decrease(to, relaxedDist);
                }
                visitor.EdgeRelaxed(edgeId);
            }
        }
        colors[from] = Color::BLACK;
        visitor.FinishVertex(from);
    }
}

template <class Queue, class Visitor>
auto BoostDijkstra(
    const Graph& graph,
    const std::vector<VertexId>& sources,
    Visitor& visitor)
{
    std::vector<Weight> distances(graph.VerticesCount(), Dijkstra::INF);
    std::vector<Color> colors(graph.VerticesCount(), Color::WHITE);
    Queue queue;
    BoostDijkstra(graph, distances, colors, sources, queue, visitor);
    return distances;
}
