#include "contraction.h"
#include "multilevel_graphs.h"

#include <iostream>

static inline constexpr auto I = Dijkstra::INF;

void TestMultilevelDijkstra() {
    auto mlg = BuildTestMlg();

    auto contracted = SimpleContraction(mlg);

    std::vector<std::vector<Weight>> answer{
        {0, 65, 48, 52, 32, 1},
        {I, 0,  2,  6,  10, I},
        {I, I,  0,  4,  8,  I},
        {I, I,  I,  0,  I,  I},
        {I, I,  16, 20, 0,  I},
        {I, 64, 66, 70, 74, 0},
    };

    auto numVertices = mlg.VerticesCount(0);
    for (VertexId from = 0; from < numVertices; ++from) {
        for (VertexId to = 0; to < numVertices; ++to) {
            auto distances = MultiLevelDijkstra<StdHeap>(contracted, {from}, {to});
            ASSERT_EQUAL(distances[to], answer[from][to]);
        }
    }
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestMultilevelDijkstra);
    std::cerr << "Done tests.\n";
}
