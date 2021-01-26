#include "algorithms/bidirectional_dijkstra.h"
#include "algorithms/dijkstra.h"
#include "algorithms/shortest_path_algorithm.h"
#include "graph/serializer.h"
#include "utils.h"

int main() {
    const std::string path = "../graphs/USA-road-d.NY.gr";
    auto graph = GraphDeserializer(path).DimacsDeserialize();
    Log() << graph.VerticesCount() << graph.EdgesCount();

    ShortestPathAlgorithm<Dijkstra> dijkstra(graph);
    dijkstra.Preprocess();
    dijkstra.FindShortestPathsWeights(0);

    Log() << dijkstra.GetShortestDistance(1)
        << dijkstra.GetShortestDistance(2)
        << dijkstra.GetShortestDistance(12)
        << dijkstra.GetShortestDistance(329);

    ShortestPathAlgorithm<BidirectionalDijkstra> bidijkstra(graph);
    bidijkstra.Preprocess();

    Log() << bidijkstra.FindShortestPathWeight(0, 1)
        << bidijkstra.FindShortestPathWeight(0, 2)
        << bidijkstra.FindShortestPathWeight(0, 12)
        << bidijkstra.FindShortestPathWeight(0, 329);

    return 0;
}
