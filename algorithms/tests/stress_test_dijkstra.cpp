#include "contraction.h"
#include "dijkstra.h"
//#include "dijkstra_prevrun.h"
#include "bidirectional_dijkstra.h"
#include "multilevel_dijkstra.h"
#include "serializer.h"
#include "shortest_path_algorithm.h"
#include "topology_builders.h"
#include "utils.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <filesystem>
#include <iostream>

using namespace boost;

static VertexId NUM_VERTICES = 1000;

std::string FindFile(std::filesystem::path path) {
    for (auto curDir = std::filesystem::current_path();
            curDir != "/";
            curDir = curDir.parent_path()) {
        for (const auto& p : std::filesystem::recursive_directory_iterator(curDir)) {
            if (!std::filesystem::is_directory(p)) {
                auto str = p.path().string();
                if (str.ends_with(path.string())) {
                    return str;
                }
            }
        }
    }
    return "";
}

const auto& TestGraph() {
    static const std::string path = FindFile("graphs/USA-road-d.NY.gr");
    static const auto testGraph = GraphDeserializer(path).DimacsDeserialize();
    static bool logOnce = true;
    if (logOnce) {
        Log() << "Read testGraph from" << path
              << ", vertices:" << testGraph.VerticesCount()
              << ", edges:" << testGraph.EdgesCount();
        logOnce = false;
    }
    return testGraph;
}

const std::vector<float>& CalcDistancesBoost() {
    using graph_t = adjacency_list<
        vecS, vecS, directedS, no_property, property<edge_weight_t, float>>;
    using edge_t = std::pair<int, int>;
    using vertex_descriptor = graph_traits<graph_t>::vertex_descriptor;

    static bool calculated = false;
    static std::vector<edge_t> edges;
    static std::vector<float> weights;
    if (!calculated) {
        edges.reserve(TestGraph().EdgesCount());
        weights.reserve(TestGraph().EdgesCount());
        for (VertexId u = 0; u < TestGraph().VerticesCount(); ++u) {
            for (EdgeId edgeId : TestGraph().GetOutgoingEdges(u)) {
                edges.emplace_back(u, TestGraph().GetTarget(edgeId));
                weights.push_back(TestGraph().GetEdgeProperties(edgeId).weight);
            }
        }
    }

    static graph_t g(edges.begin(), edges.end(), weights.data(), TestGraph().VerticesCount());

    static std::vector<vertex_descriptor> p(num_vertices(g));
    static std::vector<float> d(num_vertices(g));
    static vertex_descriptor s = vertex(0, g);

    if (!calculated) {
        dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));
        calculated = true;
    }
    return d;
}

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

//void TestDijkstraPrevRun() {
//    const auto& d = CalcDistancesBoost();
//
//    ShortestPathAlgorithm<PrevRunDijkstra> dijkstra(TestGraph());
//    dijkstra.Preprocess();
//    dijkstra.FindShortestPathsWeights(0);
//
//    VertexId target = TestGraph().VerticesCount() / 2;
//    while (d[target] == Dijkstra::INF) {
//        --target;
//    }
//
//    ShortestPathAlgorithm<BidirectionalDijkstra> bidijkstra(TestGraph());
//    bidijkstra.Preprocess();
//
//    for (VertexId vertex = 0; vertex < NUM_VERTICES; ++vertex) {
//        auto my = dijkstra.FindShortestPathWeight(vertex, target);
//        auto rightAnswer = bidijkstra.FindShortestPathWeight(vertex, target);
//        ASSERT(my == rightAnswer);
//    }
//}

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
        return PreprocessGraph(TestGraph(), path, levels);
    });

    Log() << "Searching paths...";
    Timer timer;
    for (VertexId vertex = 0; vertex < NUM_VERTICES; ++vertex) {
        ASSERT_EQUAL(algorithm.FindShortestPathWeight(0, vertex), d[vertex]);
    }
    Log() << "Paths were found in" << timer.ElapsedMs() << "ms";
}

int main() {
    std::cerr << "Running tests ...\n";
    TestGraph();
    CalcDistancesBoost();

    NUM_VERTICES = 1000;

    RUN_TEST(TestMultiLevelDijkstra);
    RUN_TEST(TestDijkstra);
    RUN_TEST(TestDijkstraHonest);
//    RUN_TEST(TestDijkstraPrevRun);
    RUN_TEST(TestBidirectionalDijkstra);
    std::cerr << "Done tests.\n";
}
