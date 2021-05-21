#include "bidirectional_dijkstra.h"
#include "multilevel_dijkstra.h"
#include "shortest_path_algorithm.h"
#include "topology_builders.h"
#include "utils.h"

#include "../from_boost/test_utils.h"

#include <execution>
#include <filesystem>
#include <fstream>

static constexpr VertexId NUM_VERTICES = 100;
constexpr int TESTS = 5;
using TTime = double;

static std::optional<Weight> globalWeight;
void VerifyWeight(Weight weight) {
    if (globalWeight) {
        ASSERT(globalWeight == weight);
    } else {
        globalWeight = weight;
    }
}

constexpr inline static std::string_view STATS_FOLDER = "stats";
constexpr inline static std::string_view GRAPHS_FOLDER = "dumped_graphs";

namespace StatType {
constexpr inline static std::string_view AVERAGE = "AVERAGE";
constexpr inline static std::string_view TOTAL = "TOTAL";
}

struct StatTimer {
    TTime value;
    std::string_view name;
};

struct Stats {
    std::string_view algoName;
    std::unordered_map<std::string_view, std::vector<TTime>> timers;

    void Add(const StatTimer& statTimer) {
        timers[statTimer.name].push_back(statTimer.value);
    }
};

std::ostream& operator<<(std::ostream& os, const Stats& stats) {
    os << stats.algoName << ":\n";
    os << stats.timers;
    os << std::endl;
    return os;
}

template <class R>
StatTimer CalcAverage(R&& range) {
    StatTimer statTimer;
    statTimer.name = StatType::AVERAGE;
    statTimer.value = std::reduce(std::execution::par, range.begin(), range.end(), 0.0);
    statTimer.value /= std::distance(range.begin(), range.end());
    return statTimer;
}

template <class R>
StatTimer CalcTotal(R&& range) {
    StatTimer statTimer;
    statTimer.name = StatType::TOTAL;
    statTimer.value = std::reduce(std::execution::par, range.begin(), range.end(), 0.0);
    return statTimer;
}

template <typename Algorithm>
void DumpStats(const Stats& stats) {
    std::filesystem::create_directory(STATS_FOLDER);
    std::filesystem::path path(STATS_FOLDER);
    std::stringstream ss;
    std::time_t result = std::time(nullptr);
    ss << std::asctime(std::localtime(&result)) << "_" << Algorithm::GetName();
    path /= ss.str();
    path.replace_extension(".txt");
    std::ofstream out(path);
    ASSERT(out.is_open());

    Log() << stats;
    out << stats;
}

template <typename Algorithm>
void SingleSourceSingleTarget(
    const std::vector<VertexId>& sources,
    const std::vector<VertexId>& targets,
    Stats& stats)
{
    Weight total = 0;

    ShortestPathAlgorithm<Algorithm> algorithm(TestGraph());
    algorithm.Preprocess();

    std::vector<TTime> timers(sources.size() * targets.size());
    auto timerIt = timers.begin();
    for (auto source : sources) {
        for (auto target : targets) {
            Timer timer;
            auto weight = algorithm.FindShortestPathWeight(source, target);
            *timerIt++ = timer.ElapsedMs();
            total += weight;
        }
    }

    stats.algoName = algorithm.GetName();
    stats.Add(CalcTotal(timers));
    stats.Add(CalcAverage(timers));
    Log() << stats;

    VerifyWeight(total);
}

IntermediateGraph PreprocessGraph(
    const WGraph& originalGraph,
    const std::filesystem::path& path,
    CompactTopology topology)
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
        topology);
    std::ofstream out(path, std::ios::binary);
    if (out.is_open()) {
        Log() << "Dumping to" << path;
        graph.Dump(out);
    }
    return graph;
}

struct MLDSimple : public MultilevelDijkstraAlgorithm {
    template <class ...Args>
    MLDSimple(Args&& ...args) : MultilevelDijkstraAlgorithm(std::forward<Args>(args)...) {
    }

    static constexpr std::string_view GetName() {
        return "MLDSimple";
    }

    void Preprocess() {
        std::filesystem::path path = GRAPHS_FOLDER;
        path /= "multilevel_test_graph.graph";
        const LevelId levels = 7;
        MultilevelDijkstraAlgorithm::Preprocess([&]() {
            return PreprocessGraph(TestGraph(), path, BuildSimplyTopology(TestGraph(), levels));
        });
    }
};

struct MLDInertialFlow : public MultilevelDijkstraAlgorithm {
    template <class ...Args>
    MLDInertialFlow(Args&& ...args) : MultilevelDijkstraAlgorithm(std::forward<Args>(args)...) {
    }

    static constexpr std::string_view GetName() {
        return "MLDInertialFlow";
    }

    void Preprocess() {
        std::filesystem::path path = GRAPHS_FOLDER;
        path /= "multilevel_test_graph_inertial_flow.graph";
        const LevelId levels = 7;
        MultilevelDijkstraAlgorithm::Preprocess([&]() {
            return PreprocessGraph(TestGraph(), path, BuildTopologyInertialFlow(TestCoordGraph(), levels));
        });
    }
};

using ALGORITHMS = TypeArray<BidirectionalDijkstra, MLDSimple, MLDInertialFlow>;

void Preload() {
    (void) TestCoordGraph();
    (void) TestGraph();
}

int main() {
    Log() << "Preloading ...\n";
    Preload();

    Log() << "Running tests ...\n";

    auto sources = GenerateRandomVertices(NUM_VERTICES, TestGraph().VerticesCount());
    auto targets = GenerateRandomVertices(NUM_VERTICES, TestGraph().VerticesCount());

    std::array<Stats, ALGORITHMS::size> stats;
    constexpr_for<0, ALGORITHMS::size>([&] (auto i) {
        for (int test = 0; test < TESTS; ++test) {
            SingleSourceSingleTarget<ALGORITHMS::get<i>>(sources, targets, std::get<i>(stats));
        }
    });
    constexpr_for<0, ALGORITHMS::size>([&] (auto i) {
        DumpStats<ALGORITHMS::get<i>>(std::get<i>(stats));
    });

    Log() << "Done tests.\n";
}
