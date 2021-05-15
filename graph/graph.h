#pragma once

#include "types.h"
#include "utils.h"

#include <iostream>
#include <vector>

struct Edge {
    Edge() = default;

    Edge(EdgeId id, VertexId from, VertexId to);

    EdgeId id;
    VertexId from;
    VertexId to;

    bool operator==(const Edge& rhs) const = default;
};

std::ostream& operator<<(std::ostream& os, const Edge& edge);

struct EdgeProperty {
    Weight weight;

    bool operator==(const EdgeProperty& rhs) const = default;
};

std::ostream& operator<<(std::ostream& os, const EdgeProperty& edgeProperty);

class Graph {
public:
    Graph() = default;

    void Dump(std::ostream& os) const;
    void Load(std::istream& is);

    EdgeProperty GetEdgeProperties(EdgeId edgeId) const;

    IteratorRange<std::vector<EdgeId>::const_iterator>
    GetOutgoingEdges(VertexId from) const;

    const std::vector<Edge>&
    GetEdges() const;

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
    explicit GraphBuilder(Graph graph);

    VertexId AddVertex();

    EdgeId AddEdge(VertexId from, VertexId to, EdgeProperty properties);

    Graph&& Build();

//private:
    Graph graph_;
    bool built_ = false;
};
