#pragma once

#include "dijkstra.h"
#include "graph.h"
#include "visitor.h"

class BidirectionalDijkstra {
public:
    explicit BidirectionalDijkstra(const WGraph& graph);

    Weight FindShortestPathWeight(VertexId source, VertexId target);

    static constexpr std::string_view GetName() {
        return "BidirectionalDijkstra";
    }

    Weight GetShortestDistance(VertexId) const;

    Dijkstra::Stats GetStats() const;

    void Preprocess();

private:
    const WGraph& graph_;
    const WGraph reversedGraph_;

    Dijkstra forward_;
    Dijkstra backward_;
};
