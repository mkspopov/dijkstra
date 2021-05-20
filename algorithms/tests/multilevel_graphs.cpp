#include "multilevel_graph.h"
#include "multilevel_graphs.h"

#include <array>

static constexpr std::array<std::tuple<VertexId, VertexId, Weight>, 7> EDGES{{
    {0, 5, 0b1},
    {1, 2, 0b10},
    {2, 3, 0b100},
    {2, 4, 0b1000},
    {4, 2, 0b10000},
    {0, 4, 0b100000},
    {5, 1, 0b1000000},
}};

WGraph BuildTestGraph() {
    GraphBuilder<WGraph, EdgeProperty> builder(6);
    for (auto[from, to, weight] : EDGES) {
        builder.AddEdge(from, to, {weight});
    }
    auto graph = builder.Build();
    return graph;
}

CoordGraph BuildTestCoordGraph() {
    static constexpr std::array<Point, 6> coords{{
        {0, 0},
        {0, 4},
        {2, 4},
        {3, 5},
        {1, 0},
        {0, 2},
    }};

    CoordGraph graph;
    for (auto coord : coords) {
        graph.AddVertex(coord);
    }
    for (auto[from, to, _] : EDGES) {
        graph.AddEdge(from, to);
    }
    return graph;
}

IntermediateGraph PreprocessGraph(
    const WGraph& originalGraph,
    const std::filesystem::path& path,
    CompactTopology topology)
{
    IntermediateGraph graph;
    if (!path.empty()) {
        std::ifstream in(path, std::ios::binary);
        if (in.is_open()) {
            Log() << "Loading from" << path;
            graph.Load(in);
            return graph;
        }
    }
    graph = SimpleContraction(
        originalGraph,
        topology);
    std::ofstream out(path, std::ios::binary);
    if (out.is_open()) {
        Log() << "Dumping to" << path;
        graph.Dump(out);
    }
    return graph;
}
