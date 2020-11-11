#include "algorithms/dijkstra.h"
#include "graph/serializer.h"
#include "utils.h"

int main() {
    const std::string path = "../graphs/USA-road-d.NY.gr";
    auto graph = GraphDeserializer(path).DimacsDeserialize();
    Log() << graph.VerticesCount() << graph.EdgesCount();

    Dijkstra dijkstra(graph);
    dijkstra.FindShortestPathsWeights(0);

    Log() << dijkstra.GetShortestDistance(1)
        << dijkstra.GetShortestDistance(2)
        << dijkstra.GetShortestDistance(12)
        << dijkstra.GetShortestDistance(329);
    return 0;
}
