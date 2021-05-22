#include "contraction.h"
#include "multilevel_graphs.h"
#include "topology_builders.h"

void TestCompactCliqueContraction() {
    auto check = [](const auto& graph, auto& contracted) {
        ASSERT_EQUAL(contracted.GetCellId(1, 1), 7u);
        ASSERT_EQUAL(contracted.GetCellId(5, 1), 6u);
        ASSERT_EQUAL(contracted.GetCellId(0, 2), 9u);

        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(0, 1)), std::vector<EdgeId>({}));
        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(1, 1)), std::vector<EdgeId>({9}));
        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(2, 1)), std::vector<EdgeId>({7, 8}));
        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(4, 1)), std::vector<EdgeId>({10}));
        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(5, 1)), std::vector<EdgeId>({11}));

        ASSERT_EQUAL(contracted.GetEdgeProperties(0), graph.GetEdgeProperties(0));
        ASSERT_EQUAL(contracted.GetEdgeProperties(1), graph.GetEdgeProperties(1));
        ASSERT_EQUAL(contracted.GetEdgeProperties(2), graph.GetEdgeProperties(2));
        ASSERT_EQUAL(contracted.GetEdgeProperties(3), graph.GetEdgeProperties(3));
        ASSERT_EQUAL(contracted.GetEdgeProperties(4), graph.GetEdgeProperties(4));
        ASSERT_EQUAL(contracted.GetEdgeProperties(5), graph.GetEdgeProperties(5));
        ASSERT_EQUAL(contracted.GetEdgeProperties(6), graph.GetEdgeProperties(6));

        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 1), 1);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 5), 0);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(0, 2), 1);

        ASSERT_EQUAL(contracted.VerticesCount(), 6u);
    };

    auto graph = BuildTestGraph();
    {
        auto topology = BuildSimplyTopology(graph, 4);
        auto contracted = MultithreadContraction(graph, topology);
        check(graph, contracted);
        ASSERT_EQUAL(contracted.GetCellId(0, 3), 11u);
        ASSERT_EQUAL(ToVector(contracted.GetOutgoingEdges(2, 2)), std::vector<EdgeId>({12}));
        ASSERT_EQUAL(contracted.LevelsCount(), 4);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(1, 3), 2);
        ASSERT_EQUAL(contracted.MaxDistinctLevel(2, 3), 2);
    }
    {
        auto topology = BuildSimplyTopology(graph, 2);
        auto contracted = MultithreadContraction(graph, topology);
        check(graph, contracted);
        ASSERT_EQUAL(contracted.LevelsCount(), 2);
    }
}

int main() {
    Log() << "Running tests ...\n";
    RUN_TEST(TestCompactCliqueContraction);
    Log() << "Done tests.\n";
}
