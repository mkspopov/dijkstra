//
// Created by mkspopov on 11.11.2020.
//

#ifndef DIJKSTRA_GRAPH_H
#define DIJKSTRA_GRAPH_H

#include "../types.h"
#include "../utils.h"

#include <vector>

struct Edge {
    Edge(VertexId to);

    VertexId to;
};

struct EdgeProperty {
    Weight weight;

    bool operator==(const EdgeProperty& rhs) const {
        return weight == rhs.weight;
    }
};

class Graph {
public:
    Graph() = default;

    EdgeProperty GetEdgeProperties(EdgeId edgeId) const;

    IteratorRange<std::vector<EdgeId>::const_iterator>
    GetOutgoingEdges(VertexId from) const;

    VertexId GetTarget(EdgeId edgeId) const;

    VertexId VerticesCount() const;

    EdgeId EdgesCount() const;

    Graph Reversed() const;

private:
    friend class GraphBuilder;

    std::vector<Edge> edges_;
    std::vector<EdgeProperty> edgeProperties_;
    std::vector<std::vector<EdgeId>> adjacencyList_;
};

class GraphBuilder {
public:
    GraphBuilder() = default;

    explicit GraphBuilder(VertexId verticesCount);

    VertexId AddVertex();

    EdgeId AddEdge(VertexId from, VertexId to, EdgeProperty properties);

    Graph&& Build();

private:
    Graph graph_;
    bool built_ = false;
};

#endif //DIJKSTRA_GRAPH_H
