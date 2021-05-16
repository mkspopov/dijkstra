#include "contraction.h"
#include "multilevel_dijkstra.h"
#include "topology_builders.h"

MultilevelDijkstraAlgorithm::MultilevelDijkstraAlgorithm(const WeightGraph<EdgeProperty>& graph)
    : Dijkstra(graph)
{}

Weight MultilevelDijkstraAlgorithm::FindShortestPathWeight(VertexId source, VertexId target) {
    InitSearch(source);
    auto& dijkstraVisitor = *this;
    MultilevelDijkstra<Finish::FIRST_TARGET>(graph_, distances_, colors_, {source}, {target}, AllTransitions{}, heap_, dijkstraVisitor);
    return distances_[target];
}

const WeightGraph<EdgeProperty>& MultilevelDijkstraAlgorithm::GetOriginalGraph() const {
    return Dijkstra::graph_;
}

void MultilevelDijkstraAlgorithm::Preprocess(const std::filesystem::path& path, LevelId levels) {
    Dijkstra::Preprocess();

    if (!path.empty()) {
        std::ifstream in(path, std::ios::binary);
        if (in.is_open()) {
            Log() << "Loading from" << path;
            graph_.Load(in);
            return;
        }
    }
    graph_ = SimpleContraction(
        GetOriginalGraph(),
        BuildSimplyTopology(GetOriginalGraph(), levels));
    std::ofstream out(path, std::ios::binary);
    if (out.is_open()) {
        Log() << "Dumping to" << path;
        graph_.Dump(out);
    }
}
