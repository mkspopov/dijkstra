#include "../../algorithms/contraction.h"
#include "multilevel_graph.h"

void TestMultilevelGraph() {
    const std::vector<std::tuple<VertexId, VertexId, Weight>> edges{
        {0, 1,        10},
        {1, 2,        20},
        {2, 3,        30},
        {2, 4,        40},
        {4, 2,        50},
        {0, 4,        60},
    };

    Topology topology{
        {
            {0, 1, 2, 3, 4},
            {0, 0, 1, 2, 1},
            {0, 0, 0, 0, 0},
        },
        {0, 5, 8, 9}
    };

    GraphBuilder builder(5);
    for (auto [from, to, weight] : edges) {
        builder.AddEdge(from, to, {weight});
    }
    auto graph = builder.Build();

    auto mlg = MultilevelGraph(graph, topology);
    assert(mlg.LevelsCount() == 3);

    auto contracted = CliqueContraction(mlg);
    assert(contracted.LevelsCount() == 3);
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestMultilevelGraph);
    std::cerr << "Done tests.\n";
}
