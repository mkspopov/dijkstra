#include "bidirectional_dijkstra.h"
#include "multilevel_dijkstra.h"
#include "shortest_path_algorithm.h"
#include "topology_builders.h"
#include "utils.h"

#include "../from_boost/utils_for_tests.h"

#include <execution>
#include <filesystem>
#include <fstream>

using TTime = double;

struct TestInstance {
    void Init(int argc, char* argv[]) {
        if (argc > 1) {
            GRAPHS_FOLDER = argv[1];
        }
        if (argc > 2) {
            NUM_VERTICES = std::strtoul(argv[2], nullptr, 10);
        }
        if (argc > 3) {
            STATS_FOLDER = argv[3];
        }
        if (argc > 4) {
            TESTS = std::strtol(argv[4], nullptr, 10);
        }

        Log() << "GRAPHS_FOLDER" << GRAPHS_FOLDER;
        Log() << "NUM_VERTICES" << NUM_VERTICES;
        Log() << "STATS_FOLDER" << STATS_FOLDER;
        Log() << "TESTS" << TESTS << '\n';
    }

    void VerifyWeight(Weight weight) {
        if (globalWeight) {
            if (*globalWeight != weight) {
                throw std::runtime_error("Bug in VerifyWeight!");
            }
        } else {
            globalWeight = weight;
        }
    }

    std::optional<Weight> globalWeight;

    std::filesystem::path GRAPHS_FOLDER = "../../graphs/";
    VertexId NUM_VERTICES = 5;
    std::filesystem::path STATS_FOLDER = "stats";
    int TESTS = 5;

    static constexpr inline std::string_view INERTIAL_FLOW = "inertial_flow";
    static constexpr inline std::string_view SIMPLE_PARTITION = "simple";
};

auto& TI() {
    static TestInstance ti;
    return ti;
}

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

static std::filesystem::path currentGraphName;
static CoordGraph coordGraph;
static WGraph distanceGraph;

void Preload(std::filesystem::path graphPath) {
    currentGraphName = graphPath.filename();
    distanceGraph = GraphDeserializer(graphPath).ReadDistances();
    Log() << "Read testGraph"
          << ", vertices:" << distanceGraph.VerticesCount()
          << ", edges:" << distanceGraph.EdgesCount();

    coordGraph = GraphDeserializer(graphPath).ReadCoordinates();
    Log() << "Read testGraph"
          << ", vertices:" << coordGraph.VerticesCount()
          << ", edges:" << coordGraph.EdgesCount();
}

template <typename Algorithm>
void DumpStats(const Stats& stats, Dijkstra::Stats& dijkstraStats) {
    std::filesystem::create_directory(TI().STATS_FOLDER);
    std::filesystem::path path(TI().STATS_FOLDER);
    std::stringstream ss;
    std::time_t result = std::time(nullptr);
    std::string time = std::asctime(std::localtime(&result));
    time.pop_back();
    ss << time << "_" << currentGraphName << "_" << Algorithm::GetName();
    path /= ss.str();
    path.replace_extension(".txt");
    std::ofstream out(path);
    ASSERT(out.is_open());

    Log() << stats;
    Log() << dijkstraStats / TI().TESTS;
    out << stats << std::endl;
    auto numQueries = TI().TESTS * TI().NUM_VERTICES * TI().NUM_VERTICES;
    out << dijkstraStats / numQueries << std::endl;
}

const auto& GetCoordGraph() {
    return coordGraph;
}

const auto& GetGraph() {
    return distanceGraph;
}

template <typename Algorithm>
void SingleSourceSingleTarget(
    const std::vector<VertexId>& sources,
    const std::vector<VertexId>& targets,
    Stats& stats,
    Dijkstra::Stats& dijkstraStats)
{
    Weight total = 0;

    ShortestPathAlgorithm<Algorithm> algorithm(GetGraph());
    algorithm.Preprocess();
    for (int test = 0; test < TI().TESTS; ++test) {
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
        dijkstraStats += algorithm.GetStats();
        Log() << stats;
    }

    TI().VerifyWeight(total);
}

template <class TopologyBuilder>
IntermediateGraph PreprocessGraph(
    const WGraph& originalGraph,
    const std::filesystem::path& path,
    const TopologyBuilder& topologyBuilder)
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
    graph = MultithreadContraction(
        originalGraph,
        topologyBuilder());
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
        std::filesystem::path path = TI().GRAPHS_FOLDER;
        path /= "dumped";
        path /= TI().SIMPLE_PARTITION;
        std::filesystem::create_directories(path);
        path /= currentGraphName;
        const LevelId levels = 7;
        MultilevelDijkstraAlgorithm::Preprocess([&]() {
            return PreprocessGraph(GetGraph(), path, [&]() {
                return BuildSimplyTopology(GetGraph(), levels);
            });
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
        std::filesystem::path path = TI().GRAPHS_FOLDER;
        path /= "dumped";
        path /= TI().INERTIAL_FLOW;
        std::filesystem::create_directories(path);
        path /= currentGraphName;
        const LevelId levels = 7;
        MultilevelDijkstraAlgorithm::Preprocess([&]() {
            return PreprocessGraph(GetGraph(), path, [&]() {
                return BuildInertialFlow(GetCoordGraph(), levels);
            });
        });
    }
};

using ALGORITHMS = TypeArray<Dijkstra, BidirectionalDijkstra, MLDSimple, MLDInertialFlow>;

void PerfQueries() {
    for (const auto& entry : std::filesystem::directory_iterator(TI().GRAPHS_FOLDER)) {
        Log() << entry;
        auto graphPath = std::filesystem::path(entry);
        if (graphPath.extension() == ".gr") {
            Log() << "Preloading ...\n";
            Preload(graphPath);

            auto sources = GenerateRandomVertices(TI().NUM_VERTICES, GetGraph().VerticesCount());
            auto targets = GenerateRandomVertices(TI().NUM_VERTICES, GetGraph().VerticesCount());

            std::array<Stats, ALGORITHMS::size> stats;
            std::array<Dijkstra::Stats, ALGORITHMS::size> dijkstraStats;

            constexpr_for<0, ALGORITHMS::size>([&] (auto i) {
                SingleSourceSingleTarget<ALGORITHMS::get<i>>(
                    sources,
                    targets,
                    std::get<i>(stats),
                    std::get<i>(dijkstraStats));
            });
            Log() << "Final stats:";
            constexpr_for<0, ALGORITHMS::size>([&] (auto i) {
                DumpStats<ALGORITHMS::get<i>>(std::get<i>(stats), std::get<i>(dijkstraStats));
            });

            TI().globalWeight = std::nullopt;
        }
    }
}

//template <class Graph>
void CountCutEdges(const IntermediateGraph& graph, const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path);
    ASSERT(file.is_open());
    file << graph.stats_;
}

void PerfLevels() {
    std::filesystem::path statsDir = TI().STATS_FOLDER;
    statsDir /= "cut_edges";
    for (const auto& entry : std::filesystem::directory_iterator(TI().GRAPHS_FOLDER)) {
        Log() << entry;
        auto graphPath = std::filesystem::path(entry);
        if (graphPath.extension() == ".gr") {
            Log() << "Preloading ...\n";
            Preload(graphPath);
            constexpr LevelId levels = 7;
            std::filesystem::path dumpDir = TI().GRAPHS_FOLDER;
            dumpDir /= "dumped";

            {
                auto path = dumpDir / TI().SIMPLE_PARTITION;
                std::filesystem::create_directories(path);
                path /= currentGraphName;
                auto graph = PreprocessGraph(GetGraph(), path, [&]() {
                    return BuildSimplyTopology(GetGraph(), levels);
                });
                CountCutEdges(graph, statsDir / TI().SIMPLE_PARTITION / currentGraphName);
            }

            {
                auto path = dumpDir / TI().INERTIAL_FLOW;
                std::filesystem::create_directories(path);
                path /= currentGraphName;
                auto graph = PreprocessGraph(GetGraph(), path, [&]() {
                    return BuildInertialFlow(GetCoordGraph(), levels);
                });
                CountCutEdges(graph, statsDir / TI().INERTIAL_FLOW / currentGraphName);
            }
        }
    }
}

int main(int argc, char* argv[]) {
    TI().Init(argc, argv);

    Log() << "Running tests ...\n";

    PerfQueries();
//    PerfLevels();

    Log() << "Done tests.\n";
}
