#include "contraction.h"
#include "multilevel_graphs.h"
#include "multilevel_dijkstra.h"
#include "shortest_path_algorithm.h"
#include "topology_builders.h"

#include <iostream>

static inline constexpr auto I = Dijkstra::INF;

//void TestMultilevelDijkstraOld() {
//    auto mlg = BuildTestMlg();
//
//    auto contracted = SimpleContraction(mlg);
//
//    std::vector<std::vector<Weight>> answer{
//        {0, 65, 48, 52, 32, 1},
//        {I, 0,  2,  6,  10, I},
//        {I, I,  0,  4,  8,  I},
//        {I, I,  I,  0,  I,  I},
//        {I, I,  16, 20, 0,  I},
//        {I, 64, 66, 70, 74, 0},
//    };
//
//    auto numVertices = mlg.VerticesCount(0);
//    for (VertexId from = 0; from < numVertices; ++from) {
//        for (VertexId to = 0; to < numVertices; ++to) {
//            auto distances = MultilevelDijkstra<StdHeap>(contracted, {from}, {to});
//            ASSERT_EQUAL(distances[to], answer[from][to]);
//        }
//    }
//}

void TestMultilevelDijkstra() {
    auto graph = BuildTestGraph();
    const LevelId levels = 4;
    auto topology = BuildSimplyTopology(graph, levels);
    auto contracted = SimpleContraction(graph, topology);

    std::vector<std::vector<Weight>> answer{
        {0, 65, 48, 52, 32, 1},
        {I, 0,  2,  6,  10, I},
        {I, I,  0,  4,  8,  I},
        {I, I,  I,  0,  I,  I},
        {I, I,  16, 20, 0,  I},
        {I, 64, 66, 70, 74, 0},
    };

    auto numVertices = graph.VerticesCount();
    for (VertexId from = 0; from < numVertices; ++from) {
        for (VertexId to = 0; to < numVertices; ++to) {
            auto distances = MultilevelDijkstra<StdHeap>(contracted, {from}, {to});
            ASSERT_EQUAL(distances[to], answer[from][to]);
        }
    }
}

void TestMultiLevelDijkstraAlgorithm() {
    std::filesystem::path path = "/tmp/tests/TestMultiLevelDijkstraAlgorithm.graph";
    auto originalGraph = BuildTestGraph();
    const LevelId levels = 4;
    ShortestPathAlgorithm<MultilevelDijkstraAlgorithm> algorithm(originalGraph);
    Log() << "Preprocessing...";
    algorithm.Preprocess([&]() {
        return PreprocessGraph(originalGraph, path, BuildSimplyTopology(originalGraph, levels));
    });

    // Who is responsible for grpah preprocessing?
    // MultilevelDijkstra -> ShortestPathAlgorithm
    //

    std::vector<std::vector<Weight>> answer{
        {0, 65, 48, 52, 32, 1},
        {I, 0,  2,  6,  10, I},
        {I, I,  0,  4,  8,  I},
        {I, I,  I,  0,  I,  I},
        {I, I,  16, 20, 0,  I},
        {I, 64, 66, 70, 74, 0},
    };

    Log() << "Searching paths...";
    auto numVertices = originalGraph.VerticesCount();
    for (VertexId from = 0; from < numVertices; ++from) {
        for (VertexId to = 0; to < numVertices; ++to) {
            ASSERT_EQUAL(algorithm.FindShortestPathWeight(from, to), answer[from][to]);
        }
    }
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestMultilevelDijkstra);
    RUN_TEST(TestMultiLevelDijkstraAlgorithm);
    std::cerr << "Done tests.\n";
}
