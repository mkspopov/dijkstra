#include "graph.h"
#include "utils.h"

#include <fstream>

Graph BuildTestGraph() {
    const std::vector<std::tuple<VertexId, VertexId, Weight>> edges{
        {0, 5, 0b1},
        {1, 2, 0b10},
        {2, 3, 0b100},
        {2, 4, 0b1000},
        {4, 2, 0b10000},
        {0, 4, 0b100000},
        {5, 1, 0b1000000},
    };

    GraphBuilder builder(6);
    for (auto[from, to, weight] : edges) {
        builder.AddEdge(from, to, {weight});
    }
    auto graph = builder.Build();
    return graph;
}

void TestDumpAndLoad() {
    const auto graph = BuildTestGraph();
    {
        std::ofstream out("/tmp/TestDumpAndLoad.graph");
        graph.Dump(out);
        graph.Dump(out);
    }
    Graph loaded;
    {
        std::ifstream in("/tmp/TestDumpAndLoad.graph");
        Log() << in.tellg();
        loaded.Load(in);
        Log() << in.tellg();
        loaded.Load(in);
    }
    ASSERT_EQUAL(loaded.EdgesCount(), graph.EdgesCount());
    ASSERT_EQUAL(loaded.GetEdges(), graph.GetEdges());
    ASSERT_EQUAL(loaded.VerticesCount(), graph.VerticesCount());
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestDumpAndLoad);
    std::cerr << "Done tests.\n";
}
