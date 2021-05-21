#pragma once

#include "dijkstra.h"
#include "visitor.h"
#include "graph.h"

class BidirectionalDijkstra {
public:
    explicit BidirectionalDijkstra(const WGraph& graph);

    Weight FindShortestPathWeight(VertexId source, VertexId target);

    static constexpr std::string_view GetName() {
        return "BidirectionalDijkstra";
    }

    Weight GetShortestDistance(VertexId) const;

    [[nodiscard]] bool OptimizedForward(VertexId source) const;

    void Preprocess();

private:
    const WGraph& graph_;
    const WGraph reversedGraph_;

    Dijkstra forward_;
    Dijkstra backward_;
};
