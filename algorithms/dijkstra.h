//
// Created by mkspopov on 11.11.2020.
//

#ifndef DIJKSTRA_DIJKSTRA_H
#define DIJKSTRA_DIJKSTRA_H

#include "heap.h"
#include "../graph/graph.h"

#include <cassert>
#include <limits>
#include <unordered_set>
#include <vector>

class Dijkstra {
    static constexpr Weight INF = std::numeric_limits<Weight>::infinity();
    static constexpr Weight START_WEIGHT = 0;

public:
    struct HeapElement {
        HeapElement(VertexId vertex, Weight weight)
            : vertex(vertex)
            , weight(weight) {
        }

        bool operator>(const HeapElement& rhs) const {
            if (weight == rhs.weight) {
                return vertex > rhs.vertex;
            }
            return weight > rhs.weight;
        }

        VertexId vertex;
        Weight weight;
    };

    explicit Dijkstra(const Graph& graph)
            : graph_(graph)
            , distances_(graph_.VerticesCount(), INF) {
        affectedVertices_.reserve(graph_.VerticesCount());
    }

    void FindShortestPathsWeights(VertexId source) {
        Clear();

        DiscoverVertex(source);
        distances_[source] = START_WEIGHT;
        heap_.emplace(source, START_WEIGHT);

        while (!heap_.empty()) {
            const auto [from, fromDistance] = heap_.top();
            heap_.pop();
            ExamineVertex(from);

            if (fromDistance > distances_[from]) {
                continue;
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
     * Extract the shortest distance after FindShortestPathsWeights.
     */
    Weight GetShortestDistance(VertexId target) const {
        return distances_[target];
    }

private:
    void NotRelaxed(EdgeId edgeId) {
    }
    void RelaxEdge(EdgeId edgeId) {
    }
    void ExamineEdge(EdgeId edgeId) {
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
