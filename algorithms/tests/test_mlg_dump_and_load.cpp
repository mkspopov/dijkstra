#include "contraction.h"
#include "multilevel_graphs.h"
#include "topology_builders.h"
#include "utils.h"

#include <fstream>

void TestDumpAndLoad() {
    const auto path = "/tmp/tests/TestMlgDumpAndLoad.graph";
    auto graph = BuildTestGraph();
    const LevelId levels = 4;
    auto topology = BuildSimplyTopology(graph, levels);
    auto contracted = SimpleContraction(graph, topology);
    {
        std::ofstream out(path, std::ios::binary);
        contracted.Dump(out);
        contracted.Dump(out);
    }
    IntermediateGraph loaded;
    {
        std::ifstream in(path, std::ios::binary);
        loaded.Load(in);
        loaded.Load(in);
    }
    ASSERT_EQUAL(loaded.LevelsCount(), contracted.LevelsCount());
    ASSERT_EQUAL(loaded.VerticesCount(), contracted.VerticesCount());
    ASSERT_EQUAL(loaded.builder_.graph_.GetEdges(), loaded.builder_.graph_.GetEdges());
    ASSERT_EQUAL(loaded.vertices_, loaded.vertices_);
    ASSERT_EQUAL(loaded.topology_.parents_, loaded.topology_.parents_);
}

int main() {
    Log() << "Running tests ...\n";
    RUN_TEST(TestDumpAndLoad);
    Log() << "Done tests.\n";
}
