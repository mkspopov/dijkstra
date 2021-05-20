#include "flow_network.h"
#include "graph_traverse.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <tuple>
#include <unordered_set>
#include <vector>

template <class GraphType>
class ShortestPathBfsVisitor {
public:
    ShortestPathBfsVisitor(
        const GraphType& graph,
        VertexId source,
        VertexId target,
        std::vector<EdgeId>* path)
        : graph_(graph)
        , source_(source)
        , target_(target)
        , parent_(graph_.VerticesCount())
        , path_(path)
    {}

    void DiscoverVertex(VertexId vertex) {
        if (vertex == target_) {
            while (vertex != source_) {
                auto parentEdgeId = parent_[vertex];
                path_->push_back(parentEdgeId);
                vertex = graph_.GetSource(parentEdgeId);
            }
            std::reverse(path_->begin(), path_->end());
        }
    }

    void ExamineEdge(EdgeId) {
    }

    void TreeEdge(EdgeId edgeId) {
        parent_[graph_.GetTarget(edgeId)] = edgeId;
    }

private:
    const GraphType& graph_;
    const VertexId source_ = 0;
    const VertexId target_ = 0;
    std::vector<EdgeId> parent_;
    std::vector<EdgeId>* path_ = nullptr;
};

void FlowNetwork::AddFlow(EdgeId edgeId, int flow) {
    edgeProperties_[edgeId].flow += flow;
    edgeProperties_[BackwardEdgeId(edgeId)].flow -= flow;
}

int FlowNetwork::GetFlow() const {
    int flow = 0;
    for (auto edgeId : GetOutgoingEdges(source_)) {
        flow += edgeProperties_[edgeId].flow;
    }
    return flow;
}

VertexId FlowNetwork::GetFlowSource() const {
    return source_;
}

VertexId FlowNetwork::GetSink() const {
    return sink_;
}

int FlowNetwork::ResidualCapacity(EdgeId edgeId) const {
    return edgeProperties_[edgeId].capacity - edgeProperties_[edgeId].flow;
}

EdgeId FlowNetwork::BackwardEdgeId(EdgeId edgeId) const {
    return edgeId ^ static_cast<EdgeId>(1);
}

void FlowNetworkBuilder::AddEdgeWithCapacity(const Edge& edge, int capacity) {
    builder_.AddEdge(edge.from, edge.to, FlowProperties{capacity});
    builder_.AddEdge(edge.to, edge.from, FlowProperties{0});
}

FlowNetwork&& FlowNetworkBuilder::Build() {
    return builder_.Build();
}

void FlowNetworkBuilder::SetSink(VertexId sink) {
    builder_.graph_.sink_ = sink;
}

void FlowNetworkBuilder::SetSource(VertexId source) {
    builder_.graph_.source_ = source;
}

void FlowNetworkBuilder::SetVerticesCount(VertexId verticesCount) {
    for (VertexId vertex = 0; vertex < verticesCount; ++vertex) {
        builder_.AddVertex();
    }
}

EdmondsKarp::EdmondsKarp(FlowNetwork* network) : network_(network) {
}

int EdmondsKarp::FindMaxFlow() {
    auto source = network_->GetFlowSource();
    auto sink = network_->GetSink();
    std::vector<EdgeId> path;

    const auto& residualNetwork = network_->ResidualNetworkView();
    TraverseGraphInBfsOrder(
        residualNetwork,
        source,
        ShortestPathBfsVisitor(residualNetwork, source, sink, &path));
    while (!path.empty()) {
        UpdateFlow(path);
        path.clear();
        TraverseGraphInBfsOrder(
            residualNetwork,
            source,
            ShortestPathBfsVisitor(residualNetwork, source, sink, &path));
    }
    return network_->GetFlow();
}

void EdmondsKarp::UpdateFlow(const std::vector<EdgeId>& path) {
    auto minCapacity_edgeId = *std::min_element(
        path.begin(),
        path.end(),
        [this](EdgeId lhs, EdgeId rhs) {
            return network_->ResidualCapacity(lhs) < network_->ResidualCapacity(rhs);
        });

    auto minCapacity = network_->ResidualCapacity(minCapacity_edgeId);
    for (auto edgeId : path) {
        network_->AddFlow(edgeId, minCapacity);
    }
}
