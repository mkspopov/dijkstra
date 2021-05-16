//
// Created by mkspopov on 13.11.2020.
//

#pragma once

#include "graph.h"

template <class Algorithm>
class ShortestPathAlgorithm {
public:
    template <class G>
    explicit ShortestPathAlgorithm(const G& graph)
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

    template <class ...Args>
    void Preprocess(Args&& ...args) {
        algorithm_.Preprocess(std::forward<Args>(args)...);
    }

    std::string GetName() const {
        return algorithm_.GetName();
    }

private:
    Algorithm algorithm_;
};
