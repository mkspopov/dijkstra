#include "contraction.h"
#include "multilevel_graphs.h"
#include "topology_builders.h"

void TestMultilevelGraph() {
    auto mlg = BuildTestMlg();
    {
        ASSERT_EQUAL(mlg.GetCellId(1), 6);
        ASSERT_EQUAL(mlg.GetCellId(1, 1), 6);
        ASSERT_EQUAL(mlg.GetCellId(5), 6);
        ASSERT_EQUAL(mlg.GetCellId(5, 1), 6);
        ASSERT_EQUAL(mlg.GetCellId(6, 1), 6);
        ASSERT_EQUAL(mlg.GetCellId(6, 2), 9);
        ASSERT_EQUAL(mlg.GetCellId(0, 2), 9);
        ASSERT_EQUAL(mlg.GetCellId(0, 3), 11);
        ASSERT_EQUAL(mlg.GetCellId(6, 3), 11);

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

/*
void TestCliqueContraction() {
    auto mlg = BuildTestMlg();

    auto contracted = SimpleContraction(mlg);
    {
        ASSERT_EQUAL(contracted.GetCellId(1, 1), 6);
        ASSERT_EQUAL(contracted.GetCellId(5, 1), 6);
        ASSERT_EQUAL(contracted.GetCellId(0, 2), 9);
        ASSERT_EQUAL(contracted.GetCellId(0, 3), 11);

//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(0)), std::vector<EdgeId>({0, 5, 10}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(1)), std::vector<EdgeId>({1}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(2)), std::vector<EdgeId>({2, 3}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(3)), std::vector<EdgeId>({}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(4)), std::vector<EdgeId>({4}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(5)), std::vector<EdgeId>({6}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(6)), std::vector<EdgeId>({8, 9}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(7)), std::vector<EdgeId>({7}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(8)), std::vector<EdgeId>({}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(9)), std::vector<EdgeId>({11, 12}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(10)), std::vector<EdgeId>({}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(11)), std::vector<EdgeId>({}));

        ASSERT_EQUAL(contracted.GetEdgeProperties(0), mlg.GetEdgeProperties(0));
        ASSERT_EQUAL(contracted.GetEdgeProperties(1), mlg.GetEdgeProperties(1));
        ASSERT_EQUAL(contracted.GetEdgeProperties(2), mlg.GetEdgeProperties(2));
        ASSERT_EQUAL(contracted.GetEdgeProperties(3), mlg.GetEdgeProperties(3));
        ASSERT_EQUAL(contracted.GetEdgeProperties(4), mlg.GetEdgeProperties(4));
        ASSERT_EQUAL(contracted.GetEdgeProperties(5), mlg.GetEdgeProperties(5));
        ASSERT_EQUAL(contracted.GetEdgeProperties(6), mlg.GetEdgeProperties(6));
//        ASSERT_EQUAL(contracted.GetEdgeProperties(7), mlg.GetEdgeProperties(2));
//        ASSERT_EQUAL(contracted.GetEdgeProperties(8), mlg.GetEdgeProperties(5));
//        ASSERT_EQUAL(contracted.GetEdgeProperties(9), mlg.GetEdgeProperties(1));
//        ASSERT_EQUAL(contracted.GetEdgeProperties(10), EdgeProperty{mlg.GetEdgeProperties(0).weight + mlg.GetEdgeProperties(6).weight});
//        ASSERT_EQUAL(contracted.GetEdgeProperties(10), EdgeProperty{0b1000001});
//        ASSERT_EQUAL(contracted.GetEdgeProperties(11), mlg.GetEdgeProperties(1));
//        ASSERT_EQUAL(contracted.GetEdgeProperties(12), mlg.GetEdgeProperties(5));

//        ASSERT_EQUAL(contracted.GetVertices(0), std::vector<VertexId>());
//        ASSERT_EQUAL(contracted.GetVertices(5), std::vector<VertexId>());
//        ASSERT_EQUAL(contracted.GetVertices(6), std::vector<VertexId>({0, 1, 5}));
//        ASSERT_EQUAL(contracted.GetVertices(10), std::vector<VertexId>({7, 8}));
//        ASSERT_EQUAL(contracted.GetVertices(11), std::vector<VertexId>({9, 10}));

        ASSERT_EQUAL(contracted.LevelsCount(), 4);

        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 1), 0);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 5), 0);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 2), 2);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(1, 3), 2);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(2, 3), 1);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 6), 0);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 7), 2);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 10), 2);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(6, 8), 2);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(6, 9), 0);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(11, 0), 0);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(11, 7), 0);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(11, 10), 0);

        ASSERT_EQUAL(contracted.VerticesCount(), 6);

//        ASSERT_EQUAL(contracted.VerticesCount(0), 6);
//        ASSERT_EQUAL(contracted.VerticesCount(1), 3);
//        ASSERT_EQUAL(contracted.VerticesCount(2), 2);
//        ASSERT_EQUAL(contracted.VerticesCount(3), 1);
    }
}
*/

void TestCompactCliqueContraction() {
    auto graph = BuildTestGraph();
    auto [topGraph, topology] = BuildSimplyTopology(graph, 4);
    CompactMultilevelGraph mlg(graph, topGraph, topology);
    auto contracted = SimpleContraction(mlg);

    {
        ASSERT_EQUAL(contracted.GetCellId(1, 1), 7);
        ASSERT_EQUAL(contracted.GetCellId(5, 1), 6);
        ASSERT_EQUAL(contracted.GetCellId(0, 2), 9);
        ASSERT_EQUAL(contracted.GetCellId(0, 3), 11);

        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(0, 1)), std::vector<EdgeId>({}));
        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(1, 1)), std::vector<EdgeId>({11}));
        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(2, 2)), std::vector<EdgeId>({12}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(2)), std::vector<EdgeId>({2, 3}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(3)), std::vector<EdgeId>({}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(4)), std::vector<EdgeId>({4}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(5)), std::vector<EdgeId>({6}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(6)), std::vector<EdgeId>({8, 9}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(7)), std::vector<EdgeId>({7}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(8)), std::vector<EdgeId>({}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(9)), std::vector<EdgeId>({11, 12}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(10)), std::vector<EdgeId>({}));
//        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(11)), std::vector<EdgeId>({}));

        ASSERT_EQUAL(contracted.GetEdgeProperties(0), mlg.GetEdgeProperties(0));
        ASSERT_EQUAL(contracted.GetEdgeProperties(1), mlg.GetEdgeProperties(1));
        ASSERT_EQUAL(contracted.GetEdgeProperties(2), mlg.GetEdgeProperties(2));
        ASSERT_EQUAL(contracted.GetEdgeProperties(3), mlg.GetEdgeProperties(3));
        ASSERT_EQUAL(contracted.GetEdgeProperties(4), mlg.GetEdgeProperties(4));
        ASSERT_EQUAL(contracted.GetEdgeProperties(5), mlg.GetEdgeProperties(5));
        ASSERT_EQUAL(contracted.GetEdgeProperties(6), mlg.GetEdgeProperties(6));
//        ASSERT_EQUAL(contracted.GetEdgeProperties(7), mlg.GetEdgeProperties(2));
//        ASSERT_EQUAL(contracted.GetEdgeProperties(8), mlg.GetEdgeProperties(5));
//        ASSERT_EQUAL(contracted.GetEdgeProperties(9), mlg.GetEdgeProperties(1));
//        ASSERT_EQUAL(contracted.GetEdgeProperties(10), EdgeProperty{mlg.GetEdgeProperties(0).weight + mlg.GetEdgeProperties(6).weight});
//        ASSERT_EQUAL(contracted.GetEdgeProperties(10), EdgeProperty{0b1000001});
//        ASSERT_EQUAL(contracted.GetEdgeProperties(11), mlg.GetEdgeProperties(1));
//        ASSERT_EQUAL(contracted.GetEdgeProperties(12), mlg.GetEdgeProperties(5));

//        ASSERT_EQUAL(contracted.GetVertices(0), std::vector<VertexId>());
//        ASSERT_EQUAL(contracted.GetVertices(5), std::vector<VertexId>());
//        ASSERT_EQUAL(contracted.GetVertices(6), std::vector<VertexId>({0, 1, 5}));
//        ASSERT_EQUAL(contracted.GetVertices(10), std::vector<VertexId>({7, 8}));
//        ASSERT_EQUAL(contracted.GetVertices(11), std::vector<VertexId>({9, 10}));

        ASSERT_EQUAL(contracted.LevelsCount(), 4);

        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 1), 1);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 5), 0);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 2), 1);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(1, 3), 2);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(2, 3), 2);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 6), 0);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 7), 2);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 10), 2);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(6, 8), 2);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(6, 9), 0);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(11, 0), 0);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(11, 7), 0);
//        ASSERT_EQUAL(contracted.MaxDistinctLevel(11, 10), 0);

        ASSERT_EQUAL(contracted.VerticesCount(), 6);

//        ASSERT_EQUAL(contracted.VerticesCount(0), 6);
//        ASSERT_EQUAL(contracted.VerticesCount(1), 3);
//        ASSERT_EQUAL(contracted.VerticesCount(2), 2);
//        ASSERT_EQUAL(contracted.VerticesCount(3), 1);
    }
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestMultilevelGraph);
//    RUN_TEST(TestCliqueContraction);
    RUN_TEST(TestCompactCliqueContraction);
    std::cerr << "Done tests.\n";
}
