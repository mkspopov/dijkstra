#include "../algorithms/shortest_path_algorithm.h"
#include "../algorithms/dijkstra.h"
#include "../algorithms/bidirectional_dijkstra.h"
#include "../graph/serializer.h"
#include "../utils/utils.h"
#include "utils.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <filesystem>

using namespace boost;

const std::string testGraphPath = "../../graphs/USA-road-d.NY.gr";
const auto testGraph = GraphDeserializer(testGraphPath).DimacsDeserialize();

const std::vector<float>& CalcDistancesBoost() {
    using graph_t = adjacency_list<
        vecS, vecS, directedS, no_property, property<edge_weight_t, float>>;
    using edge_t = std::pair<int, int>;
    using vertex_descriptor = graph_traits<graph_t>::vertex_descriptor;

    static bool calculated = false;
    static std::vector<edge_t> edges;
    static std::vector<float> weights;
    if (!calculated) {
        edges.reserve(testGraph.EdgesCount());
        weights.reserve(testGraph.EdgesCount());
        for (VertexId u = 0; u < testGraph.VerticesCount(); ++u) {
            for (EdgeId edgeId : testGraph.GetOutgoingEdges(u)) {
                edges.emplace_back(u, testGraph.GetTarget(edgeId));
                weights.push_back(testGraph.GetEdgeProperties(edgeId).weight);
            }
        }
    }

    static graph_t g(edges.begin(), edges.end(), weights.data(), testGraph.VerticesCount());

    static std::vector<vertex_descriptor> p(num_vertices(g));
    static std::vector<float> d(num_vertices(g));
    static vertex_descriptor s = vertex(0, g);

    if (!calculated) {
        dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));
        calculated = true;
    }
    return d;
}

constexpr int TESTS = 1000;

void TestDijkstra() {
    const auto& d = CalcDistancesBoost();

    ShortestPathAlgorithm<Dijkstra> dijkstra(testGraph);
    dijkstra.Preprocess();

    int test = 0;
    for (VertexId vertex = 0; vertex < testGraph.VerticesCount(); ++vertex) {
        ASSERT(dijkstra.FindShortestPathWeight(0, vertex) == d[vertex]);
        if (++test >= TESTS) {
            break;
        }
    }
}

void TestBidirectionalDijkstra() {
    const auto& d = CalcDistancesBoost();

    ShortestPathAlgorithm<BidirectionalDijkstra> bidijkstra(testGraph);
    bidijkstra.Preprocess();

    int test = 0;
    for (VertexId vertex = 0; vertex < testGraph.VerticesCount(); ++vertex) {
        ASSERT(bidijkstra.FindShortestPathWeight(0, vertex) == d[vertex]);
        if (++test >= TESTS) {
            break;
        }
    }
}

static std::optional<Weight> globalWeight;
void VerifyWeight(Weight weight) {
    if (globalWeight) {
        ASSERT(globalWeight == weight);
    } else {
        globalWeight = weight;
    }
}

struct Stats {
    double totalTime;
    std::string_view name;
    std::unordered_map<std::string_view, double> timers;
};

namespace daw::json {
template <>
struct json_data_contract<Stats> {
    using opt_into_iostreams = void;
    using type = json_member_list<
        json_number<"totalTime", double>
        , json_string_raw<"name", std::string_view>
        , json_key_value<
            "statss", std::unordered_map<std::string_view, double>,
            std::double_t,
        std::string_view>
//            json_number<no_name, std::double_t, LiteralAsStringOpt::Always>>
    >;

    static constexpr auto to_json_data(Stats const& v) {
//        return std::forward_as_tuple(v.totalTime, v.name);
        return std::forward_as_tuple(v.totalTime, v.name, v.timers);
    }
};
} // namespace daw::json


constexpr inline static std::string_view STATS_FOLDER = "stats";

template <typename Algorithm>
void DumpStats(Algorithm& algorithm, const std::vector<uint64_t>& timers) {
    std::filesystem::create_directory(STATS_FOLDER);
    auto currentTimestamp = std::chrono::high_resolution_clock::now();
    std::filesystem::path path(STATS_FOLDER);
    path.append(std::to_string(currentTimestamp.time_since_epoch().count()));
    path.replace_extension(".txt");
    std::ofstream out(path);

    Stats stats;
    stats.totalTime = std::reduce(timers.begin(), timers.end(), 0);
    stats.name = algorithm.GetName();
    stats.timers["Average"] = stats.totalTime / timers.size();

    out << stats << std::endl;

    out.write(reinterpret_cast<const char*>(timers.data()), timers.size() * sizeof(uint64_t));
}

template <typename Algorithm>
void SingleSourceSingleTarget(
    Algorithm& algorithm,
    const std::vector<VertexId>& sources,
    const std::vector<VertexId>& targets)
{
    Weight total = 0;

    std::vector<uint64_t> timers(sources.size() * targets.size());
    auto timerIt = timers.begin();
    for (auto source : sources) {
        for (auto target : targets) {
            Timer timer;
            auto weight = algorithm.FindShortestPathWeight(source, target);
            *timerIt++ = timer.Elapsed();
            total += weight;
        }
    }
    DumpStats(algorithm, timers);
    VerifyWeight(total);
}

int main() {
    std::cerr << "Running tests ...\n";

    ShortestPathAlgorithm<BidirectionalDijkstra> bidijkstra(testGraph);
    bidijkstra.Preprocess();
    auto sources = GenerateRandomVertices(5, testGraph.VerticesCount());
    auto targets = GenerateRandomVertices(5, testGraph.VerticesCount());
    SingleSourceSingleTarget(bidijkstra, sources, targets);
    SingleSourceSingleTarget(bidijkstra, sources, targets);


//    RUN_TEST(TestDijkstra);
//    RUN_TEST(TestBidirectionalDijkstra);
    std::cerr << "Done tests.\n";
}
