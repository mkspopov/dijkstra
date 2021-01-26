#include "../algorithms/shortest_path_algorithm.h"
#include "../algorithms/dijkstra.h"
#include "../algorithms/bidirectional_dijkstra.h"
#include "../graph/serializer.h"
#include "../utils.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace boost;

const std::string path = "../../graphs/USA-road-d.NY.gr";
const auto testGraph = GraphDeserializer(path).DimacsDeserialize();

const std::vector<float>& CalcDistancesBoost() {
    using graph_t = adjacency_list<
        vecS, vecS, directedS, no_property, property<edge_weight_t, float>>;
    using edge_t = std::pair<int, int>;
    using vertex_descriptor = graph_traits<graph_t>::vertex_descriptor;

    static bool calculated = false;
    static std::vector<edge_t> edges;
    static std::vector<float> weights;
    if (!calculated) {
        edges.reserve(testGraph.EdgesCount());
        weights.reserve(testGraph.EdgesCount());
        for (VertexId u = 0; u < testGraph.VerticesCount(); ++u) {
            for (EdgeId edgeId : testGraph.GetOutgoingEdges(u)) {
                edges.emplace_back(u, testGraph.GetTarget(edgeId));
                weights.push_back(testGraph.GetEdgeProperties(edgeId).weight);
            }
        }
    }

    static graph_t g(edges.begin(), edges.end(), weights.data(), testGraph.VerticesCount());

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

    ShortestPathAlgorithm<Dijkstra> dijkstra(testGraph);
    dijkstra.Preprocess();
    dijkstra.FindShortestPathsWeights(0);

    for (int vertex = 0; vertex < testGraph.VerticesCount(); ++vertex) {
        ASSERT(dijkstra.GetShortestDistance(vertex) == d[vertex]);
    }
}

void TestBidirectionalDijkstra() {
    const auto& d = CalcDistancesBoost();

    ShortestPathAlgorithm<BidirectionalDijkstra> bidijkstra(testGraph);
    bidijkstra.Preprocess();

    for (int vertex = 0; vertex < testGraph.VerticesCount(); ++vertex) {
        ASSERT(bidijkstra.FindShortestPathWeight(0, vertex) == d[vertex]);
    }
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestDijkstra);
    RUN_TEST(TestBidirectionalDijkstra);
    std::cerr << "Done tests.\n";
}
