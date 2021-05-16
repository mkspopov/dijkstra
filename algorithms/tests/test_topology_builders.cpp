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
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestSimpleBuilder);
    std::cerr << "Done tests.\n";
}
