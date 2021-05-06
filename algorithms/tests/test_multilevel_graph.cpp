#include "contraction.h"
#include "multilevel_graph.h"

void TestMultilevelGraph() {
    const std::vector<std::tuple<VertexId, VertexId, Weight>> edges{
        {0, 5,        10},
        {1, 2,        20},
        {2, 3,        30},
        {2, 4,        40},
        {4, 2,        50},
        {0, 4,        60},
        {5, 1,        70},
    };

    Topology topology{
        {
            {0, 1, 2, 3, 4, 5},
            {0, 0, 1, 2, 1, 0},
            {0, 0, 1, 1, 1, 0},
            {0, 0, 0, 0, 0, 0},
        },
        {0, 6, 9, 11, 12}
    };

    GraphBuilder builder(6);
    for (auto [from, to, weight] : edges) {
        builder.AddEdge(from, to, {weight});
    }
    auto graph = builder.Build();

    auto mlg = MultilevelGraph(graph, topology);
    {
        ASSERT_EQUAL(mlg.LevelsCount(), 4);

        ASSERT_EQUAL(mlg.VerticesCount(), 6);
        ASSERT_EQUAL(mlg.VerticesCount(0), 6);
        ASSERT_EQUAL(mlg.VerticesCount(1), 3);
        ASSERT_EQUAL(mlg.VerticesCount(2), 2);
        ASSERT_EQUAL(mlg.VerticesCount(3), 1);

        ASSERT_EQUAL(mlg.GetCellId(1), 6);
        ASSERT_EQUAL(mlg.GetCellId(1, 1), 6);
        ASSERT_EQUAL(mlg.GetCellId(5), 6);
        ASSERT_EQUAL(mlg.GetCellId(5, 1), 6);
        ASSERT_EQUAL(mlg.GetCellId(6, 1), 6);

        ASSERT_EQUAL(mlg.GetCellId(6), 9);
        ASSERT_EQUAL(mlg.GetCellId(7), 10);
        ASSERT_EQUAL(mlg.GetCellId(8), 10);
        ASSERT_EQUAL(mlg.GetCellId(6, 1), 6);
        ASSERT_EQUAL(mlg.GetCellId(7, 1), 7);
        ASSERT_EQUAL(mlg.GetCellId(8, 1), 8);
    }



    auto contracted = CliqueContraction(mlg);
    ASSERT_EQUAL(contracted.LevelsCount(), 3);
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestMultilevelGraph);
    std::cerr << "Done tests.\n";
}
