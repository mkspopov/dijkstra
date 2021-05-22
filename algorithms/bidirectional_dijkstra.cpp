#include "bidirectional_dijkstra.h"

BidirectionalDijkstra::BidirectionalDijkstra(const WGraph& graph)
    : graph_(graph)
    , reversedGraph_(graph_.Reversed())
    , forward_(graph_)
    , backward_(reversedGraph_)
{
    forward_.Preprocess();
    backward_.Preprocess();
}

Weight BidirectionalDijkstra::FindShortestPathWeight(VertexId source, VertexId target) {
    if (source == target) {
        return 0;
    }
    forward_.InitSearch(source);
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

Weight BidirectionalDijkstra::GetShortestDistance(VertexId) const {
    return Dijkstra::INF;
}

Dijkstra::Stats BidirectionalDijkstra::GetStats() const {
    auto stats = forward_.GetStats();
    stats += backward_.GetStats();
    return stats;
}

void BidirectionalDijkstra::Preprocess() {
}
