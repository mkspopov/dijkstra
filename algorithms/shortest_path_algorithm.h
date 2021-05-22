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

    constexpr std::string_view GetName() const {
        return algorithm_.GetName();
    }

    auto GetStats() const {
        return algorithm_.GetStats();
    }

private:
    Algorithm algorithm_;
};
