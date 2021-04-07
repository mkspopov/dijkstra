//
// Created by mkspopov on 07.04.2021.
//

#pragma once

#include "dijkstra.h"
#include "heap.h"
#include "../graph/graph.h"
#include "visitor.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <memory>
#include <unordered_set>
#include <vector>

class PrevRunDijkstra {
public:
    static constexpr Weight INF = std::numeric_limits<Weight>::infinity();
    static constexpr Weight START_WEIGHT = 0;
    static constexpr VertexId UNDEFINED_VERTEX = std::numeric_limits<VertexId>::max();

    explicit PrevRunDijkstra(const Graph& graph)
        : graph_(graph)
    {
    }

    auto AffectedVertices() {
        return IteratorRange(affectedVertices_.begin(), affectedVertices_.end());
    }

    bool IsAffected(VertexId vertexId) const {
        return distances_[vertexId] != INF;
    }

    void Preprocess() {
        distances_.resize(graph_.VerticesCount(), INF);
        parents_.resize(graph_.VerticesCount(), -1);
        affectedVertices_.reserve(graph_.VerticesCount());
    }

    Weight FindShortestPathWeight(VertexId source, VertexId target) {
        InitSearch(source, target);
        while (!heap_.empty()) {
            const auto [from, fromDistance] = heap_.top();
            heap_.pop();

            if (from == target) {
                return fromDistance;
            }

            if (CalculatedOnPreviousRun(from)) {
                VertexId f = from;
                distances_[target] = std::min(
                    distances_[target],
                    distances_[f] + prevRunDistances_[target] - prevRunDistances_[f]);
                continue;
            }

            ProcessVertex(from, fromDistance);
        }
        return distances_[target];
    }

    void FindShortestPathsWeights(VertexId source) {
        InitSearch(source);
        while (!heap_.empty()) {
            const auto [from, fromDistance] = heap_.top();
            heap_.pop();
            ProcessVertex(from, fromDistance);
        }
        prevRunDistances_ = distances_;
        prevRunParents_ = parents_;
        prevRunSource_ = source;
    }

    void ProcessVertex(int from, float fromDistance) {
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
                parents_[to] = from;
                heap_.emplace(to, distance);
            } else if (distances_[to] > distance) {
                distances_[to] = distance;
                parents_[to] = from;
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
        return prevRunDistances_[target];
    }

    void InitSearch(VertexId source) {
        Clear();

        DiscoverVertex(source);
        distances_[source] = START_WEIGHT;
        parents_[source] = source;
        heap_.emplace(source, START_WEIGHT);
    }

    void InitSearch(VertexId source, VertexId target) {
        Clear();

        DiscoverVertex(target);
        DiscoverVertex(source);
        distances_[source] = START_WEIGHT;
        parents_[source] = source;
        heap_.emplace(source, START_WEIGHT);

        if (!prevRunDistances_.empty()) {
            prevRunShortestPath_.clear();
            auto cur = target;
            while (cur != prevRunSource_) {
                ASSERT(!prevRunShortestPath_.contains(cur));
                prevRunShortestPath_.insert(cur);
                cur = prevRunParents_[cur];
            }
        }
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

    bool CalculatedOnPreviousRun(VertexId vertexId) const {
        return prevRunShortestPath_.contains(vertexId);
    }

    void Clear() {
        for (const auto vertex : affectedVertices_) {
            distances_[vertex] = INF;
        }
        affectedVertices_.clear();
        ClearHeap();
    }

    void ClearHeap() {
        while (!heap_.empty()) {
            heap_.pop();
        }
    }

    const Graph& graph_;
    std::vector<Weight> distances_;
    std::vector<VertexId> affectedVertices_;
    Heap<HeapElement> heap_;

    std::unordered_set<VertexId> targets_;

    std::vector<VertexId> parents_;
    std::vector<VertexId> prevRunParents_;
    std::vector<Weight> prevRunDistances_;
    std::unordered_set<VertexId> prevRunShortestPath_;
    VertexId prevRunSource_ = 0;
};
