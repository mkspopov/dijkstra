//
// Created by mkspopov on 13.11.2020.
//

#pragma once

#include "dijkstra.h"
#include "visitor.h"
#include "../graph/graph.h"


class BidirectionalDijkstra {
public:
    explicit BidirectionalDijkstra(const Graph& graph)
        : graph_(graph)
        , reversedGraph_(graph_.Reversed())
        , forward_(graph_)
        , backward_(reversedGraph_)
    {
        forward_.Preprocess();
        backward_.Preprocess();
    }

    Weight FindShortestPathWeight(VertexId source, VertexId target) {
        if (source == target) {
            return 0;
        }
        if (!OptimizedForward(source)) {
            forward_.InitSearch(source);
        } else if (forward_.IsProcessed(target)) {
            return forward_.GetShortestDistance(target);
        }
        backward_.InitSearch(target);

        VertexId prevProcessedForward = source;
        VertexId prevProcessedBackward = target;
        bool isForwardTurn = true;
        while (!forward_.IsProcessed(prevProcessedBackward) &&
               !backward_.IsProcessed(prevProcessedForward)) {
            if (isForwardTurn) {
                prevProcessedForward = forward_.ProcessVertex();
            } else {
                prevProcessedBackward = backward_.ProcessVertex();
            }
            isForwardTurn = !isForwardTurn;
        }

        if (forward_.IsProcessed(target)) {
            return forward_.GetShortestDistance(target);
        }

        if (prevProcessedBackward == UNDEFINED && prevProcessedForward == UNDEFINED) {
            return Dijkstra::INF;
        }

        Weight minDistance = Dijkstra::INF;
        for (const auto& from : forward_.AffectedVertices()) {
            for (const auto& edgeId : graph_.GetOutgoingEdges(from)) {
                const auto to = graph_.GetTarget(edgeId);
                if (backward_.IsProcessed(to)) {
                    const auto distance = forward_.GetShortestDistance(from) +
                        graph_.GetEdgeProperties(edgeId).weight +
                        backward_.GetShortestDistance(to);
                    if (distance < minDistance) {
                        minDistance = distance;
                    }
                }
            }
        }

        return minDistance;
    }

    Weight GetShortestDistance(VertexId ) const {
        return Dijkstra::INF;
    }

    void Preprocess() {
    }

    [[nodiscard]] bool OptimizedForward(VertexId source) const {
        static VertexId prevSource = UNDEFINED;
        if (prevSource == source) {
            return true;
        }
        prevSource = source;
        return false;
    }

    std::string GetName() const {
        return "BidirectionalDijkstra";
    }

private:
    const Graph& graph_;
    const Graph reversedGraph_;

    Dijkstra forward_;
    Dijkstra backward_;

//    std::vector<Weight> distances_;
//    std::vector<VertexId> affectedVertices_;
//    Heap<HeapElement> heap_;
//
//    std::unordered_set<VertexId> targets_;
};
