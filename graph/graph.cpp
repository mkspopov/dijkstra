//
// Created by mkspopov on 11.11.2020.
//

#include "graph.h"
//#include "graph.pb.h"


#include <cassert>

std::ostream& operator<<(std::ostream& os, const Edge& edge) {
    os << '(' << edge.id << ' ' << edge.from << ' ' << edge.to << ')';
    return os;
}

std::ostream& operator<<(std::ostream& os, const EdgeProperty& edgeProperty) {
    os << edgeProperty.weight;
    return os;
}

void Graph::Dump(std::ostream& os) const {
    ::Dump(os, edges_);
    ::Dump(os, edgeProperties_);
    ::Dump(os, adjacencyList_);
//    ToPb().SerializeToOstream(&os);
}
//
//void Graph::ToPb() const {
//    graph::Graph graph;
//    for (const auto& edge : edges_) {
//        auto pbEdge = graph.add_edges();
//        pbEdge->set_id(edge.id);
//        pbEdge->set_from(edge.from);
//        pbEdge->set_to(edge.to);
//    }
//    for (const auto& [weight] : edgeProperties_) {
//        graph.add_edge_properties()->set_weight(weight);
//    }
//    for (const auto& adj : adjacencyList_) {
//        *graph.add_adj_list()->mutable_out_edge_ids() = {adj.begin(), adj.end()};
//    }
//    return graph;
//}

void Graph::Load(std::istream& is) {
    ::Load(is, edges_);
    ::Load(is, edgeProperties_);
    ::Load(is, adjacencyList_);
//    graph::Graph graph;
//    graph.ParseFromIstream(&is);
//    GraphBuilder builder(graph.adj_list_size());
//    for (const auto& edge : graph.edges()) {
//        builder.AddEdge(
//            edge.from(),
//            edge.to(),
//            EdgeProperty{graph.edge_properties(edge.id()).weight()});
//    }
//    *this = builder.Build();
}

EdgeProperty Graph::GetEdgeProperties(EdgeId edgeId) const {
    return edgeProperties_[edgeId];
}

IteratorRange<std::vector<EdgeId>::const_iterator> Graph::GetOutgoingEdges(VertexId from) const {
    return IteratorRange(
            adjacencyList_.at(from).begin(),
            adjacencyList_.at(from).end());
}

const std::vector<Edge>& Graph::GetEdges() const {
    return edges_;
}

VertexId Graph::GetTarget(EdgeId edgeId) const {
    return edges_[edgeId].to;
}

VertexId Graph::VerticesCount() const {
    return adjacencyList_.size();
}

EdgeId Graph::EdgesCount() const {
    return edges_.size();
}

Graph Graph::Reversed() const {
    GraphBuilder builder;
    for (size_t i = 0; i < adjacencyList_.size(); ++i) {
        builder.AddVertex();
    }
    for (size_t from = 0; from < adjacencyList_.size(); ++from) {
        for (const auto edgeId : GetOutgoingEdges(from)) {
            builder.AddEdge(GetTarget(edgeId), from, edgeProperties_[edgeId]);
        }
    }
    return builder.Build();
}

GraphBuilder::GraphBuilder(VertexId verticesCount) {
    graph_.adjacencyList_.resize(verticesCount);
}

GraphBuilder::GraphBuilder(Graph graph) : graph_(std::move(graph)) {
}

VertexId GraphBuilder::AddVertex() {
    graph_.adjacencyList_.emplace_back();
    return graph_.adjacencyList_.size() - 1;
}

EdgeId GraphBuilder::AddEdge(VertexId from, VertexId to, EdgeProperty properties) {
    for (auto edgeId : graph_.GetOutgoingEdges(from)) {
        if (graph_.GetTarget(edgeId) == to && graph_.GetEdgeProperties(edgeId) == properties) {
            return UNDEFINED;
        }
    }

    const EdgeId edgeId = graph_.edges_.size();
    graph_.edges_.emplace_back(edgeId, from, to);
    graph_.edgeProperties_.emplace_back(std::move(properties));
    graph_.adjacencyList_[from].push_back(edgeId);
    return edgeId;
}

Graph&& GraphBuilder::Build() {
    assert(!built_);
    built_ = true;
    return std::move(graph_);
}
