#include "graph.h"

Edge::Edge(EdgeId id, VertexId from, VertexId to) : id(id), from(from), to(to) {
}

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
    ::Dump(os, adjacencyList_);
}

void Graph::Load(std::istream& is) {
    ::Load(is, edges_);
    ::Load(is, adjacencyList_);
}

IteratorRange<std::vector<EdgeId>::const_iterator> Graph::GetOutgoingEdges(VertexId from) const {
    return IteratorRange(
            adjacencyList_.at(from).begin(),
            adjacencyList_.at(from).end());
}

const std::vector<Edge>& Graph::GetEdges() const {
    return edges_;
}

VertexId Graph::GetSource(EdgeId edgeId) const {
    return edges_[edgeId].from;
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

VertexId Graph::AddVertex() {
    adjacencyList_.emplace_back();
    return adjacencyList_.size() - 1;
}

EdgeId Graph::AddEdge(VertexId from, VertexId to) {
    const EdgeId edgeId = edges_.size();
    edges_.emplace_back(edgeId, from, to);
    adjacencyList_[from].push_back(edgeId);
    return edgeId;
}
