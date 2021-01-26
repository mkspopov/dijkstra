//
// Created by mkspopov on 13.11.2020.
//

#ifndef DIJKSTRA_BIDIRECTIONAL_DIJKSTRA_H
#define DIJKSTRA_BIDIRECTIONAL_DIJKSTRA_H

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
        forward_.InitSearch(source);
        backward_.InitSearch(target);
        std::unordered_set<VertexId> processedForward;
        std::unordered_set<VertexId> processedBackward;

        VertexId prevProcessedBackward = target;
        VertexId prevProcessedForward = source;
        bool isForwardTurn = true;
        while (!processedForward.contains(prevProcessedBackward) &&
                !processedBackward.contains(prevProcessedForward)) {
            if (isForwardTurn) {
                prevProcessedForward = forward_.ProcessVertex();
                processedForward.insert(prevProcessedForward);
            } else {
                prevProcessedBackward = backward_.ProcessVertex();
                processedBackward.insert(prevProcessedBackward);
            }
            isForwardTurn = !isForwardTurn;
        }

        if (prevProcessedBackward == Dijkstra::UNDEFINED_VERTEX && prevProcessedForward == Dijkstra::UNDEFINED_VERTEX) {
            return Dijkstra::INF;
        }

        Weight minDistance = Dijkstra::INF;
        for (const auto& from : forward_.AffectedVertices()) {
            for (const auto& edgeId : graph_.GetOutgoingEdges(from)) {
                const auto to = graph_.GetTarget(edgeId);
                if (processedBackward.contains(to)) {
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

private:
    const Graph& graph_;
    const Graph reversedGraph_;

    Dijkstra forward_;
    Dijkstra backward_;

    std::vector<Weight> distances_;
    std::vector<VertexId> affectedVertices_;
    Heap<HeapElement> heap_;

    std::unordered_set<VertexId> targets_;
};


#endif //DIJKSTRA_BIDIRECTIONAL_DIJKSTRA_H