#include "min_cut.h"
#include "multilevel_graphs.h"
#include "topology_builders.h"
#include "utils.h"

#include <set>

void TestMinCut() {
    const auto graph = BuildTestGraph();
    {
        auto minCut = FindMinCut({4, 5}, {2, 3}, graph);
        std::sort(minCut.edges.begin(), minCut.edges.end());
        ASSERT_EQUAL(minCut.edges, std::vector<EdgeId>({3, 4, 6}));
        auto answer = ContainerCast<std::vector<VertexId>>(
            ContainerCast<std::set<VertexId>>(minCut.first));
        ASSERT_EQUAL(answer, std::vector<VertexId>({0, 4, 5}));
    }
    {
        auto minCut = FindMinCut({0}, {3}, graph);
        std::sort(minCut.edges.begin(), minCut.edges.end());
        ASSERT_EQUAL(minCut.edges, std::vector<EdgeId>({2}));
        auto answer = ContainerCast<std::vector<VertexId>>(
            ContainerCast<std::set<VertexId>>(minCut.first));
        ASSERT_EQUAL(answer, std::vector<VertexId>({0, 1, 2, 4, 5}));
    }
}

void TestSmallInertialFlow() {
    const auto graph = BuildTestCoordGraph();
    auto topology = BuildTopologyInertialFlow(graph, 2, 0.4);

    ASSERT_EQUAL(topology.parents_, std::vector<VertexId>({6, 7, 9, 9, 8, 7, 10, 10, 11, 11, 12, 12}));
    ASSERT_EQUAL(topology.Level(0), 0);
    ASSERT_EQUAL(topology.Level(1), 0);
    ASSERT_EQUAL(topology.Level(2), 0);
    ASSERT_EQUAL(topology.Level(3), 0);
    ASSERT_EQUAL(topology.Level(4), 0);
    ASSERT_EQUAL(topology.Level(5), 0);
    ASSERT_EQUAL(topology.Level(6), 1);
    ASSERT_EQUAL(topology.Level(7), 1);
    ASSERT_EQUAL(topology.Level(8), 1);
    ASSERT_EQUAL(topology.Level(9), 1);
    ASSERT_EQUAL(topology.Level(10), 2);
    ASSERT_EQUAL(topology.Level(11), 2);
    ASSERT_EQUAL(topology.Level(12), 3);
}

int main() {
    Log() << "Running tests ...\n";
    RUN_TEST(TestMinCut);
    RUN_TEST(TestSmallInertialFlow);
    Log() << "Done tests.\n";
}
