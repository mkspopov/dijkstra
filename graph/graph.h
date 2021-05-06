//
// Created by mkspopov on 11.11.2020.
//

#pragma once

#include "types.h"
#include "utils.h"

#include <vector>

struct Edge {
    Edge(EdgeId id, VertexId from, VertexId to) : id(id), from(from), to(to) {
    }

    EdgeId id;
    VertexId from;
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
    explicit GraphBuilder(Graph&& graph);

    VertexId AddVertex();

    EdgeId AddEdge(VertexId from, VertexId to, EdgeProperty properties);

    Graph&& Build();

//private:
    Graph graph_;
    bool built_ = false;
};
