#include "multilevel_graphs.h"
#include "topology_builders.h"

void TestSimpleBuilder() {
    const auto graph = BuildTestGraph();
    auto topology = BuildSimplyTopology(graph, 4);
    ASSERT_EQUAL(topology.parents_.size(), 12ul);
    // 012 3  45
    // 677 8  66
    // 999 10 99
    // 11 ...
    ASSERT_EQUAL(topology.parents_, std::vector<VertexId>({6, 7, 7, 8, 6, 6, 9, 9, 10, 11, 11, 12}));
    ASSERT_EQUAL(topology.Level(0), 0);
    ASSERT_EQUAL(topology.Level(1), 0);
    ASSERT_EQUAL(topology.Level(2), 0);
    ASSERT_EQUAL(topology.Level(3), 0);
    ASSERT_EQUAL(topology.Level(4), 0);
    ASSERT_EQUAL(topology.Level(5), 0);
    ASSERT_EQUAL(topology.Level(6), 1);
    ASSERT_EQUAL(topology.Level(7), 1);
    ASSERT_EQUAL(topology.Level(8), 1);
    ASSERT_EQUAL(topology.Level(9), 2);
    ASSERT_EQUAL(topology.Level(10), 2);
    ASSERT_EQUAL(topology.Level(11), 3);
    ASSERT_EQUAL(topology.Level(12), 4);
}

int main() {
    Log() << "Running tests ...\n";
    RUN_TEST(TestSimpleBuilder);
    Log() << "Done tests.\n";
}
