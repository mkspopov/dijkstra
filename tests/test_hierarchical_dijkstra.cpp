#include "../algorithms/shortest_path_algorithm.h"
#include "../algorithms/dijkstra.h"
#include "../algorithms/bidirectional_dijkstra.h"
#include "../graph/serializer.h"
#include "../utils.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace boost;

Graph BuildHierarchicalGraph() {
    constexpr VertexId verticesCount = 10;
    GraphBuilder builder(verticesCount);
    const std::vector<std::pair<int, int>> edges{
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 4},
        {4, 5},
        {5, 6},
        {6, 7},
        {7, 8},
        {8, 9},
        {9, 10},

        {4, 6},
        {5, 7},
        {6, 5},
        {7, 6},
        {7, 4},
    };

    for (const auto& [from, to] : edges) {
        builder.AddEdge(from, to, EdgeProperty{1});
    }
    return builder.Build();
}

void TestHierarchicalDijkstra() {
    auto testGraph = BuildHierarchicalGraph();
    (void) testGraph;
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestHierarchicalDijkstra);
    std::cerr << "Done tests.\n";
}
