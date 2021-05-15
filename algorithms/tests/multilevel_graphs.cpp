#include "multilevel_graph.h"
#include "multilevel_graphs.h"

#include <vector>

Graph BuildTestGraph() {
    const std::vector<std::tuple<VertexId, VertexId, Weight>> edges{
        {0, 5, 0b1},
        {1, 2, 0b10},
        {2, 3, 0b100},
        {2, 4, 0b1000},
        {4, 2, 0b10000},
        {0, 4, 0b100000},
        {5, 1, 0b1000000},
    };

    GraphBuilder builder(6);
    for (auto[from, to, weight] : edges) {
        builder.AddEdge(from, to, {weight});
    }
    auto graph = builder.Build();
    return graph;
}

MultilevelGraph BuildTestMlg() {

    Topology topology{
        {
            {0, 1, 2, 3, 4, 5},
            {0, 0, 1, 2, 1, 0},
            {0, 0, 1, 1, 1, 0},
            {0, 0, 0, 0, 0, 0},
        },
        {0, 6, 9, 11, 12},
        {},
        {},
        {},
    };

    return MultilevelGraph(BuildTestGraph(), topology);
}
