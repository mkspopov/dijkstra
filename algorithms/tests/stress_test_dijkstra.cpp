#include "contraction.h"
#include "dijkstra.h"
#include "bidirectional_dijkstra.h"
#include "multilevel_dijkstra.h"
#include "multilevel_graphs.h"
#include "shortest_path_algorithm.h"
#include "topology_builders.h"
#include "utils.h"

#include "../from_boost/utils_for_tests.h"

#include <filesystem>

static VertexId NUM_VERTICES = 1000;

void TestDijkstra() {
    const auto& d = CalcDistancesBoost();

    ShortestPathAlgorithm<Dijkstra> dijkstra(TestGraph());
    dijkstra.Preprocess();

    dijkstra.FindShortestPathsWeights(0);

    for (VertexId vertex = 0; vertex < NUM_VERTICES; ++vertex) {
        ASSERT(dijkstra.GetShortestDistance(vertex) == d[vertex]);
    }
}

void TestDijkstraHonest() {
    const auto& d = CalcDistancesBoost();

    ShortestPathAlgorithm<Dijkstra> dijkstra(TestGraph());
    dijkstra.Preprocess();

    Timer timer;
    for (VertexId vertex = 0; vertex < NUM_VERTICES; ++vertex) {
        ASSERT(dijkstra.FindShortestPathWeight(0, vertex) == d[vertex]);
    }
    Log() << "Paths were found in" << timer.ElapsedMs() << "ms";
}

void TestBidirectionalDijkstra() {
    const auto& d = CalcDistancesBoost();

    ShortestPathAlgorithm<BidirectionalDijkstra> bidijkstra(TestGraph());
    bidijkstra.Preprocess();

    Timer timer;
    for (VertexId vertex = 0; vertex < NUM_VERTICES; ++vertex) {
        ASSERT(bidijkstra.FindShortestPathWeight(0, vertex) == d[vertex]);
    }
    Log() << "Paths were found in" << timer.ElapsedMs() << "ms";
}

void TestMultiLevelDijkstra() {
    const auto& d = CalcDistancesBoost();

    std::filesystem::path path = "/tmp/tests/multilevel_test_graph.graph";
    ShortestPathAlgorithm<MultilevelDijkstraAlgorithm> algorithm(TestGraph());
    Log() << "Preprocessing...";
    const LevelId levels = 7;
    algorithm.Preprocess([&]() {
        return PreprocessGraph(TestGraph(), path, BuildSimplyTopology(TestGraph(), levels));
    });

    Log() << "Searching paths...";
    Timer timer;
    for (VertexId vertex = 0; vertex < NUM_VERTICES; ++vertex) {
        ASSERT_EQUAL(algorithm.FindShortestPathWeight(0, vertex), d[vertex]);
    }
    Log() << "Paths were found in" << timer.ElapsedMs() << "ms";
}

void TestInertialFlow() {
    const auto& d = CalcDistancesBoost();

    std::filesystem::path path = "/tmp/tests/multilevel_test_graph_inertial_flow.graph";
    ShortestPathAlgorithm<MultilevelDijkstraAlgorithm> algorithm(TestGraph());
    Log() << "Preprocessing...";
    const LevelId levels = 7;
    algorithm.Preprocess([&]() {
        return PreprocessGraph(TestGraph(), path, BuildInertialFlow(TestCoordGraph(), levels));
    });

    Log() << "Searching paths...";
    Timer timer;
    for (VertexId vertex = 0; vertex < NUM_VERTICES; ++vertex) {
        ASSERT_EQUAL(algorithm.FindShortestPathWeight(0, vertex), d[vertex]);
    }
    Log() << "Paths were found in" << timer.ElapsedMs() << "ms";
}

int main() {
    Log() << "Running tests ...\n";
    TestGraph();
    CalcDistancesBoost();

    NUM_VERTICES = 1000;

    RUN_TEST(TestInertialFlow);
    RUN_TEST(TestMultiLevelDijkstra);
    RUN_TEST(TestDijkstra);
    RUN_TEST(TestDijkstraHonest);
    RUN_TEST(TestBidirectionalDijkstra);
    Log() << "Done tests.\n";
}
