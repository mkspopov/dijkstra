//
// Created by mkspopov on 13.11.2020.
//

#pragma once

#include "../graph/graph.h"

template <class Algorithm>
class ShortestPathAlgorithm {
public:
    explicit ShortestPathAlgorithm(const Graph& graph)
            : algorithm_(graph) {
    }

    Weight FindShortestPathWeight(VertexId source, VertexId target) {
        return algorithm_.FindShortestPathWeight(source, target);
    }

    void FindShortestPathsWeights(VertexId source) {
        algorithm_.FindShortestPathsWeights(source);
    }

    Weight GetShortestDistance(VertexId target) const {
        return algorithm_.GetShortestDistance(target);
    }

    void Preprocess() {
        algorithm_.Preprocess();
    }

private:
    Algorithm algorithm_;
};
