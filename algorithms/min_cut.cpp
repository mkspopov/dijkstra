#include "min_cut.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <tuple>
#include <vector>

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

int FlowNetwork::GetMaxPossibleFlow() const {
    int flow = 0;
    for (auto edgeId : GetOutgoingEdges(source_)) {
        flow += edgeProperties_[edgeId].capacity;
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

auto FlowNetwork::ResidualNetworkView() const {
    auto predicate = [this](EdgeId edgeId) {
        return ResidualCapacity(edgeId) > 0;
    };

    return FilteredGraph(*this, predicate);
}

EdgeId FlowNetwork::BackwardEdgeId(EdgeId edgeId) const {
    return edgeId ^ static_cast<EdgeId>(1);
}

//FlowNetwork::EdgeProperties::EdgeProperties(int capacity) : capacity(capacity) {
//}

void FlowNetworkBuilder::AddEdgeWithCapacity(const Edge& edge, int capacity) {
    builder_.AddEdge(edge.from, edge.to, FlowProperties{edge.id, capacity});
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

//int FindMinMaxGoldBars(
//    int people_number,
//    int verticesCount,
//    int source,
//    int sink,
//    const std::vector<int>& gold_bars,
//    const std::vector<Edge>& trust_relations) {
//    FlowNetworkBuilder initial_builder;
//    initial_builder.SetVerticesNumber(verticesCount);
//    initial_builder.SetSource(source);
//    initial_builder.SetSink(sink);
//    for (int human = 1; human <= people_number; ++human) {
//        initial_builder.AddEdgeWithCapacity(Edge(source, human), gold_bars[human - 1]);
//    }
//    for (const auto& edge : trust_relations) {
//        initial_builder.AddEdgeWithCapacity(edge, std::numeric_limits<int>::max());
//    }
//
//    auto has_max_flow = [&initial_builder, people_number, sink](int sink_edge_capacity) {
//        auto network_builder = initial_builder;
//        for (int human = 1; human <= people_number; ++human) {
//            network_builder.AddEdgeWithCapacity(Edge(human, sink), sink_edge_capacity);
//        }
//        auto network = network_builder.Build();
//
//        auto flow = EdmondsKarp(&network).FindMaxFlow();
//
//        return network.GetMaxPossibleFlow() == flow;
//    };
//
//    return BinarySearch(kMinPossibleAnswer, kMaxPossibleAnswer, has_max_flow);
//}
