#include "multilevel_graphs.h"
#include "topology_builders.h"

void TestSimpleBuilder() {
    const auto graph = BuildTestGraph();
    auto topology = BuildSimplyTopology(graph, 4);
    std::cerr << "a";
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestSimpleBuilder);
    std::cerr << "Done tests.\n";
}
