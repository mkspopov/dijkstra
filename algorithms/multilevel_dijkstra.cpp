#include "contraction.h"
#include "multilevel_dijkstra.h"
#include "topology_builders.h"

MultilevelDijkstraAlgorithm::MultilevelDijkstraAlgorithm(const WGraph& graph)
    : Dijkstra(graph)
{}

const WGraph& MultilevelDijkstraAlgorithm::GetOriginalGraph() const {
    return Dijkstra::graph_;
}

LevelId MultilevelDijkstraAlgorithm::LevelsCount() const {
    return graph_.LevelsCount();
}

void MultilevelDijkstraAlgorithm::Preprocess(const std::function<IntermediateGraph()>& preprocessor) {
    Dijkstra::Preprocess();
    graph_ = preprocessor();
}

IntermediateGraph PreprocessGraph(
    const WGraph& originalGraph,
    const std::filesystem::path& path,
    LevelId levels)
{
    IntermediateGraph graph;
    if (!path.empty()) {
        std::ifstream in(path, std::ios::binary);
        if (in.is_open()) {
            Log() << "Loading from" << path;
            graph.Load(in);
            return graph;
        }
    }
    graph = SimpleContraction(
        originalGraph,
        BuildSimplyTopology(originalGraph, levels));
    std::ofstream out(path, std::ios::binary);
    if (out.is_open()) {
        Log() << "Dumping to" << path;
        graph.Dump(out);
    }
    return graph;
}
