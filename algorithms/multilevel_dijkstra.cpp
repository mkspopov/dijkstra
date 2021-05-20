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
