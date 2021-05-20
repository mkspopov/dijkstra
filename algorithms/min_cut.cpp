#include "graph_traverse.h"
#include "flow_network.h"
#include "min_cut.h"

#include <iostream>
#include <unordered_set>
#include <vector>

class MinCutBfsVisitor {
public:
    explicit MinCutBfsVisitor(MinCut& minCut) : minCut_(minCut) {
    }

    void DiscoverVertex(VertexId vertex) {
        minCut_.first.insert(vertex);
    }

    void TreeEdge(EdgeId) {
    }

private:
    MinCut& minCut_;
};

MinCut FindMinCut(
    const std::vector<VertexId>& sources,
    const std::vector<VertexId>& sinks,
    const Graph& graph)
{
    FlowNetworkBuilder builder;
    builder.SetVerticesCount(graph.VerticesCount() + 2);
    auto sourceId = graph.VerticesCount();
    auto sinkId = graph.VerticesCount() + 1;
    builder.SetSource(sourceId);
    builder.SetSink(sinkId);

    const int capacity = 1;
    const int inf = std::numeric_limits<int>::max();

    for (auto edge : graph.GetEdges()) {
        builder.AddEdgeWithCapacity(edge, capacity);
    }
    for (auto edge : graph.GetEdges()) {
        builder.AddEdgeWithCapacity({UNDEFINED, edge.to, edge.from}, capacity);
    }
    for (auto source : sources) {
        builder.AddEdgeWithCapacity({UNDEFINED, sourceId, source}, inf);
    }
    for (auto sink : sinks) {
        builder.AddEdgeWithCapacity({UNDEFINED, sink, sinkId}, inf);
    }

    auto network = builder.Build();

    EdmondsKarp(&network).FindMaxFlow();

    MinCut minCut;
    TraverseGraphInBfsOrder(
        network.ResidualNetworkView(),
        sourceId,
        MinCutBfsVisitor(minCut));

    for (const auto& edge : graph.GetEdges()) {
        if ((minCut.first.contains(edge.from) && !minCut.first.contains(edge.to)) ||
            (minCut.first.contains(edge.to) && !minCut.first.contains(edge.from))) {
            minCut.edges.push_back(edge.id);
        }
    }

    return minCut;
}
