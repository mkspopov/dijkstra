#include "../algorithms/shortest_path_algorithm.h"
#include "../algorithms/dijkstra.h"
#include "../algorithms/bidirectional_dijkstra.h"
#include "../graph/serializer.h"
#include "../utils.h"

struct Topology {
    using Component = std::vector<VertexId>;
    using Layer = std::vector<Component>;

    std::vector<VertexId> zeroLayer;
    std::vector<Layer> layers;
};

class MultilayerGraph {
public:
    MultilayerGraph(Topology topology, std::vector<std::tuple<VertexId, VertexId, Weight>> edges) {
        const auto verticesCount = topology.zeroLayer.size();
        componentsOnLevel_.resize(topology.layers.size() + 1, 0);
        componentsOnLevel_.front() = verticesCount;
        componentsOnLevel_.back() = 1;
        components_.resize(
            topology.layers.size() - 1,
            std::vector<VertexId>(verticesCount));
        VertexId lastComponentNumber = verticesCount;

        GraphBuilder builder(verticesCount);
        for (size_t layerIndex = 0; layerIndex + 1 < topology.layers.size(); ++layerIndex) {
            for (const auto& component : topology.layers[layerIndex]) {
                for (auto vertexId : component) {
                    components_[layerIndex][vertexId] = lastComponentNumber;
                }
                ++lastComponentNumber;
                ++componentsOnLevel_[layerIndex + 1];
                builder.AddVertex();
            }
        }


        for (auto& [from, to, weight] : edges) {
            if (from != to) {
                builder.AddEdge(from, to, EdgeProperty{weight});
            } else {
                from = UNDEFINED;
            }
        }
        std::erase_if(edges, [](const auto& edge) {
            return edge.from == UNDEFINED;
        });

        EdgeId parentEdgeId = 0;
        for (auto [from, to, weight] : edges) {
            for (const auto& layer : components_) {
                if (layer[from] != layer[to]) {
                    auto edgeId = builder.AddEdge(layer[from], layer[to], EdgeProperty{weight});
                    if (edgeId != UNDEFINED) {
                        parentEdgeIds_[edgeId] = parentEdgeId;
                    }
                }
            }
            ++parentEdgeId;
        }

        graph_ = builder.Build();
    }

    LevelId GetLevel(EdgeId edgeId) const {
        return GetVertexLevel(GetTarget(edgeId));
    }

    LevelId GetVertexLevel(VertexId vertexId) const {
        LevelId level = 0;
        for (const int64_t compsOnLevel : componentsOnLevel_) {
            vertexId -= compsOnLevel;
            if (vertexId < 0) {
                return level;
            }
            ++level;
        }
        __builtin_unreachable();
    }

    IteratorRange<std::vector<EdgeId>::const_iterator>
    GetOutgoingEdges(VertexId from) const {
        return graph_.GetOutgoingEdges(from);
    }

    IteratorRange<std::vector<EdgeId>::const_iterator>
    GetOutgoingEdges(EdgeId fromEdge, LevelId level) const {
        return graph_.GetOutgoingEdges(GetTarget(fromEdge, level));
    }

    VertexId GetTarget(EdgeId edgeId) const {
        return graph_.GetTarget(edgeId);
    }

    VertexId GetTarget(EdgeId edgeId, LevelId level) const {
        auto to = GetTarget(edgeId);
        auto toLevel = GetVertexLevel(to);

        if (level < toLevel) {
            to = GetTarget(parentEdgeIds_[edgeId]);
            toLevel = 0;
        }
        while (toLevel < level) {
            to = components_[toLevel][to];
            ++toLevel;
        }
        return to;
    }

    Weight GetWeight(EdgeId from, EdgeId to) const {

    }

    VertexId VerticesCount() const {
        return graph_.VerticesCount();
    }

    EdgeId EdgesCount() const {
        return graph_.EdgesCount();
    }

    VertexId GetComponent(VertexId zeroLevelVertexId, size_t level) const {
        return components_[level][zeroLevelVertexId];
    }

    size_t CommonLevel(VertexId lhs, VertexId rhs) const {
        size_t level = 1;
        while (level < components_.size() && components_[level][lhs] != components_[level][rhs]) {
            lhs = components_[level][lhs];
            rhs = components_[level][rhs];
            ++level;
        }
        return level;
    }

private:
    std::vector<std::vector<VertexId>> components_;
    std::vector<size_t> componentsOnLevel_;
    std::vector<EdgeId> parentEdgeIds_;
    Graph graph_;
};

struct EdgeAndWeight {
    EdgeAndWeight(EdgeId edgeId, Weight weight)
        : edgeId(edgeId)
        , weight(weight) {
    }

    bool operator>(const EdgeAndWeight& rhs) const {
        if (weight == rhs.weight) {
            return edgeId > rhs.edgeId;
        }
        return weight > rhs.weight;
    }

    EdgeId edgeId;
    Weight weight;
};

class MultilayerDijkstra {
public:
    explicit MultilayerDijkstra(MultilayerGraph graph) : graph_(std::move(graph)) {
    }

    Weight FindShortestPathWeight(EdgeId source, EdgeId target) {
        const auto commonLevel = graph_.CommonLevel(source, target);

        Heap<EdgeAndWeight> heap;
        heap.emplace(source, 0);
        std::vector<Weight> distances(graph_.EdgesCount(), Dijkstra::INF);
        distances[source] = 0;

        while (!heap.empty()) {
            auto [edgeId, distance] = heap.top();
            if (edgeId == target) {
                return distance;
            }
            heap.pop();

            if (distances[edgeId] < distance) {
                continue;
            }

            for (auto level = graph_.GetLevel(edgeId); level < commonLevel; ++level) {
                for (auto outEdgeId : graph_.GetOutgoingEdges(edgeId, level)) {
                    // Go from edgeId to outEdgeId. Save distance to outEdgeId
                    const auto weight = graph_.GetWeight(edgeId, outEdgeId) + distance;
                    if (weight < distances[outEdgeId]) {
                        distances[outEdgeId] = weight;
                        heap.emplace(outEdgeId, weight);
                    }
                }
            }
        }
        return distances[target];
    }

private:
    MultilayerGraph graph_;
};

MultilayerGraph BuildHierarchicalGraph() {
    const std::vector<std::tuple<VertexId, VertexId, Weight>> edges{
                                 {0,       1,        1},
                                 {1,       2,        1},
                                 {2,       3,        2},
                                 {3,       4,        2},
                                 {4,       5,        2},
                                 {4,       6,        3},
                                 {5,       3,        2},
                                 {5,       6,        2},
                                 {6,       4,        5},
                                 {6,       7,        2},
                                 {7,       5,        1},
                                 {7,       8,        2},
                                 {8,       9,        1},
    };

    Topology topology{
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
        {
            {{0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9}},
            {{10, 11, 12}},
        },
    };

    return MultilayerGraph(topology, edges);
}

void TestHierarchicalDijkstra() {
    auto testGraph = BuildHierarchicalGraph();
    ASSERT(testGraph.VerticesCount() == 13);
    ASSERT(testGraph.EdgesCount() == 16);
}

int main() {
    std::cerr << "Running tests ...\n";
    RUN_TEST(TestHierarchicalDijkstra);
    std::cerr << "Done tests.\n";
}
