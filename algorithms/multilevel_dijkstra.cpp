#include "contraction.h"
#include "multilevel_dijkstra.h"
#include "topology_builders.h"

MultilevelDijkstraAlgorithm::MultilevelDijkstraAlgorithm(const Graph& graph)
    : Dijkstra(graph)
{}

Weight MultilevelDijkstraAlgorithm::FindShortestPathWeight(VertexId source, VertexId target) {
    InitSearch(source);
    MultilevelDijkstra(graph_, distances_, colors_, {source}, {target}, AllTransitions{}, heap_);
    return distances_[target];
}

const Graph& MultilevelDijkstraAlgorithm::GetOriginalGraph() const {
    return Dijkstra::graph_;
}

void MultilevelDijkstraAlgorithm::Preprocess(std::filesystem::path path) {
    Dijkstra::Preprocess();

    if (!path.empty()) {
        std::ifstream in(path, std::ios::binary);
        if (in.is_open()) {
            Log() << "Loading from" << path;
            graph_.Load(in);
            return;
        }
    }
    graph_ = IntermediateGraph(SimpleContraction(
        GetOriginalGraph(),
        BuildSimplyTopology(GetOriginalGraph(), 3).second));
    std::ofstream out(path, std::ios::binary);
    if (out.is_open()) {
        Log() << "Dumping to" << path;
        graph_.Dump(out);
    }
}
