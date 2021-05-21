#include "test_utils.h"

const std::vector<float>& CalcDistancesBoost() {
    using graph_t = adjacency_list<
        vecS, vecS, directedS, no_property, property<edge_weight_t, float>>;
    using edge_t = std::pair<int, int>;
    using vertex_descriptor = graph_traits<graph_t>::vertex_descriptor;

    static bool calculated = false;
    static std::vector<edge_t> edges;
    static std::vector<float> weights;
    if (!calculated) {
        edges.reserve(TestGraph().EdgesCount());
        weights.reserve(TestGraph().EdgesCount());
        for (VertexId u = 0; u < TestGraph().VerticesCount(); ++u) {
            for (EdgeId edgeId : TestGraph().GetOutgoingEdges(u)) {
                edges.emplace_back(u, TestGraph().GetTarget(edgeId));
                weights.push_back(TestGraph().GetEdgeProperties(edgeId).weight);
            }
        }
    }

    static graph_t g(edges.begin(), edges.end(), weights.data(), TestGraph().VerticesCount());

    static std::vector<vertex_descriptor> p(num_vertices(g));
    static std::vector<float> d(num_vertices(g));
    static vertex_descriptor s = vertex(0, g);

    if (!calculated) {
        dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));
        calculated = true;
    }
    return d;
}

std::filesystem::path FindFile(std::filesystem::path path) {
    for (auto curDir = std::filesystem::current_path();
         curDir != "/";
         curDir = curDir.parent_path()) {
        for (const auto& p : std::filesystem::recursive_directory_iterator(curDir)) {
            if (!std::filesystem::is_directory(p)) {
                auto str = p.path().string();
                if (str.ends_with(path.string())) {
                    return p.path();
                }
            }
        }
    }
    return {};
}

std::vector<VertexId> GenerateRandomVertices(int vertexCount, int maxVertex) {
    std::vector<VertexId> vertices(vertexCount);
    std::uniform_int_distribution<> dis(0, maxVertex - 1);
    std::generate_n(vertices.begin(), vertexCount, [&dis]() {
        return dis(GetRng());
    });
    return vertices;
}

const CoordGraph& TestCoordGraph() {
    static const auto path = FindFile(NEW_YORK_GRAPH);
    static const auto testGraph = GraphDeserializer(path).ReadCoordinates();
    static bool logOnce = true;
    if (logOnce) {
        Log() << "Read testGraph from" << path
              << ", vertices:" << testGraph.VerticesCount();
        logOnce = false;
    }
    return testGraph;
}

const WGraph& TestGraph() {
    static const auto path = FindFile(NEW_YORK_GRAPH);
    static const auto testGraph = GraphDeserializer(path).ReadDistances();
    static bool logOnce = true;
    if (logOnce) {
        Log() << "Read testGraph from" << path
              << ", vertices:" << testGraph.VerticesCount()
              << ", edges:" << testGraph.EdgesCount();
        logOnce = false;
    }
    return testGraph;
}
