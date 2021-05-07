#include "contraction.h"
#include "multilevel_graph.h"

auto BuildTestMlg() {
    const std::vector<std::tuple<VertexId, VertexId, Weight>> edges{
        {0, 5, 10},
        {1, 2, 20},
        {2, 3, 30},
        {2, 4, 40},
        {4, 2, 50},
        {0, 4, 60},
        {5, 1, 70},
    };

    Topology topology{
        {
            {0, 1, 2, 3, 4, 5},
               {0, 0, 1, 2, 1, 0},
                  {0, 0, 1, 1, 1, 0},
                     {0, 0, 0, 0, 0, 0},
        },
        {   0, 6, 9, 11, 12}
    };

    GraphBuilder builder(6);
    for (auto[from, to, weight] : edges) {
        builder.AddEdge(from, to, {weight});
    }
    auto graph = builder.Build();

    return MultilevelGraph(graph, topology);
}

void TestMultilevelGraph() {
    auto mlg = BuildTestMlg();
    {
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

        ASSERT_EQUAL(mlg.GetCells(0), ToVector(std::views::iota(0u, 6u)));
        ASSERT_EQUAL(mlg.GetCells(1), ToVector(std::views::iota(6u, 9u)));
        ASSERT_EQUAL(mlg.GetCells(2), ToVector(std::views::iota(9u, 11u)));
        ASSERT_EQUAL(mlg.GetCells(3), ToVector(std::views::iota(11u, 12u)));

        ASSERT_EQUAL(ToVector(mlg.GetOutgoingEdges(0)), std::vector<EdgeId>({0, 5}));
        ASSERT_EQUAL(ToVector(mlg.GetOutgoingEdges(6)), std::vector<EdgeId>({7, 9}));

        ASSERT_EQUAL(mlg.GetEdgeProperties(7), mlg.GetEdgeProperties(1));
        ASSERT_EQUAL(mlg.GetEdgeProperties(9), mlg.GetEdgeProperties(5));

        ASSERT_EQUAL(mlg.GetVertices(0), std::vector<VertexId>());
        ASSERT_EQUAL(mlg.GetVertices(5), std::vector<VertexId>());
        ASSERT_EQUAL(mlg.GetVertices(6), std::vector<VertexId>({0, 1, 5}));
        ASSERT_EQUAL(mlg.GetVertices(10), std::vector<VertexId>({7, 8}));
        ASSERT_EQUAL(mlg.GetVertices(11), std::vector<VertexId>({9, 10}));

        ASSERT_EQUAL(mlg.LevelsCount(), 4);

        ASSERT_EQUAL(mlg.MaxDistinctLevel(0, 1), 0);
        ASSERT_EQUAL(mlg.MaxDistinctLevel(0, 5), 0);
        ASSERT_EQUAL(mlg.MaxDistinctLevel(0, 6), 0);
        ASSERT_EQUAL(mlg.MaxDistinctLevel(0, 7), 2);
        ASSERT_EQUAL(mlg.MaxDistinctLevel(0, 10), 2);
        ASSERT_EQUAL(mlg.MaxDistinctLevel(6, 8), 2);
        ASSERT_EQUAL(mlg.MaxDistinctLevel(6, 9), 0);
        ASSERT_EQUAL(mlg.MaxDistinctLevel(11, 0), 0);
        ASSERT_EQUAL(mlg.MaxDistinctLevel(11, 7), 0);
        ASSERT_EQUAL(mlg.MaxDistinctLevel(11, 10), 0);

        ASSERT_EQUAL(mlg.VerticesCount(), 12);

        ASSERT_EQUAL(mlg.VerticesCount(0), 6);
        ASSERT_EQUAL(mlg.VerticesCount(1), 3);
        ASSERT_EQUAL(mlg.VerticesCount(2), 2);
        ASSERT_EQUAL(mlg.VerticesCount(3), 1);
    }
}

void TestCliqueContraction() {
    auto mlg = BuildTestMlg();

    auto contracted = CliqueContraction(mlg);
    ASSERT_EQUAL(contracted.LevelsCount(), 3);
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestMultilevelGraph);
    RUN_TEST(TestCliqueContraction);
    std::cerr << "Done tests.\n";
}
