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

    IteratorRange<std::vector<EdgeId>::const_iterator>
    GetOutgoingEdges(VertexId from) const;

    const std::vector<Edge>&
    GetEdges() const;

    VertexId GetSource(EdgeId edgeId) const;

    VertexId GetTarget(EdgeId edgeId) const;

    VertexId VerticesCount() const;

    EdgeId EdgesCount() const;

protected:
    template <class G, class ...Properties>
    friend class GraphBuilder;

    VertexId AddVertex();

    EdgeId AddEdge(VertexId from, VertexId to);

    std::vector<Edge> edges_;
    std::vector<std::vector<EdgeId>> adjacencyList_;
};

template <class EdgeProperties>
class WeightGraph : public Graph {
public:
    WeightGraph() = default;

    void Dump(std::ostream& os) const;
    void Load(std::istream& is);

    EdgeProperties GetEdgeProperties(EdgeId edgeId) const {
        return edgeProperties_.at(edgeId);
    }

    WeightGraph Reversed() const;

protected:
    template <class G, class ...Properties>
    friend class GraphBuilder;

    EdgeId AddEdge(VertexId from, VertexId to, EdgeProperties&& properties);

    std::vector<EdgeProperties> edgeProperties_;
};

template <class EdgeProperties>
EdgeId WeightGraph<EdgeProperties>::AddEdge(VertexId from, VertexId to, EdgeProperties&& properties) {
    for (auto edgeId : GetOutgoingEdges(from)) {
        if (GetTarget(edgeId) == to && GetEdgeProperties(edgeId) == properties) {
            return UNDEFINED;
        }
    }
    auto id = Graph::AddEdge(from, to);
    edgeProperties_.emplace_back(std::forward<EdgeProperties>(properties));
    return id;
}

template <class EdgeProperties>
void WeightGraph<EdgeProperties>::Dump(std::ostream& os) const {
    Graph::Dump(os);
    ::Dump(os, edgeProperties_);
}

template <class EdgeProperties>
void WeightGraph<EdgeProperties>::Load(std::istream& is) {
    Graph::Load(is);
    ::Load(is, edgeProperties_);
}

template <class G, class ...Properties>
class GraphBuilder {
public:
    GraphBuilder() = default;
    explicit GraphBuilder(VertexId verticesCount) {
        for (VertexId i = 0; i < verticesCount; ++i) {
            graph_.AddVertex();
        }
    }

    explicit GraphBuilder(G&& graph) : graph_(std::forward<G>(graph)) {
    }

    VertexId AddVertex() {
        return graph_.AddVertex();
    }

    EdgeId AddEdge(VertexId from, VertexId to, Properties&& ...properties) {
        return graph_.AddEdge(from, to, std::forward<Properties>(properties)...);
    }

    G&& Build() {
        return std::move(graph_);
    }

//private:
    G graph_;
};

template <class EdgeProperties>
WeightGraph<EdgeProperties> WeightGraph<EdgeProperties>::Reversed() const {
    GraphBuilder<WeightGraph<EdgeProperties>, EdgeProperties> builder;
    for (size_t i = 0; i < adjacencyList_.size(); ++i) {
        builder.AddVertex();
    }
    for (size_t from = 0; from < adjacencyList_.size(); ++from) {
        for (const auto edgeId : GetOutgoingEdges(from)) {
            builder.AddEdge(GetTarget(edgeId), from, GetEdgeProperties(edgeId));
        }
    }
    return builder.Build();
}

using WGraph = WeightGraph<EdgeProperty>;
