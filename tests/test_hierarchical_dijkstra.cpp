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
    MultilayerGraph(Topology topology, const std::vector<std::tuple<VertexId, VertexId, Weight>>& edges) {
        const auto verticesCount = topology.zeroLayer.size();
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
                builder.AddVertex();
            }
        }

        for (auto [from, to, weight] : edges) {
            if (from != to) {
                builder.AddEdge(from, to, EdgeProperty{weight});
            }
        }
        for (auto [from, to, weight] : edges) {
            for (const auto& layer : components_) {
                if (layer[from] != layer[to]) {
                    builder.AddEdge(layer[from], layer[to], EdgeProperty{weight});
                }
            }
        }

        graph_ = builder.Build();
    }

    VertexId GetTarget(EdgeId edgeId) const {
        return graph_.GetTarget(edgeId);
    }

    VertexId VerticesCount() const {
        return graph_.VerticesCount();
    }

    EdgeId EdgesCount() const {
        return graph_.EdgesCount();
    }

private:
    std::vector<std::vector<VertexId>> components_;
    Graph graph_;
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
