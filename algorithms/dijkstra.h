//
// Created by mkspopov on 11.11.2020.
//

#ifndef DIJKSTRA_DIJKSTRA_H
#define DIJKSTRA_DIJKSTRA_H

#include "heap.h"
#include "../graph/graph.h"
#include "visitor.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <unordered_set>
#include <vector>

class Dijkstra {
public:
    static constexpr Weight INF = std::numeric_limits<Weight>::infinity();
    static constexpr Weight START_WEIGHT = 0;
    static constexpr VertexId UNDEFINED_VERTEX = std::numeric_limits<VertexId>::max();

    explicit Dijkstra(const Graph& graph)
        : graph_(graph)
    {
    }

    auto AffectedVertices() {
        return IteratorRange(affectedVertices_.begin(), affectedVertices_.end());
    }

    void Preprocess() {
        distances_.resize(graph_.VerticesCount(), INF);
        affectedVertices_.reserve(graph_.VerticesCount());
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

    void ProcessVertex(const int from, const float fromDistance) {
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
    Heap<HeapElement> heap_;

    std::unordered_set<VertexId> targets_;
};

#endif //DIJKSTRA_DIJKSTRA_H
