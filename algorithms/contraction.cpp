#include "contraction.h"
#include "multilevel_dijkstra.h"
#include "thread_pool.h"

#include <iostream>
#include <unordered_set>
#include <vector>

IntermediateGraph::IntermediateGraph(WGraph graph, Topology topology)
    : builder_(std::move(graph))
    , topology_(std::move(topology))
    , vertices_(topology_.LevelsCount())
{
    for (VertexId v = 0; v < builder_.graph_.VerticesCount(); ++v) {
        vertices_.at(0).emplace(v, v);
    }
}

void IntermediateGraph::AddEdge(VertexId from, VertexId to, LevelId level, EdgeProperty edgeProperty) {
    ASSERT(from < VerticesCount());
    AddVertex(from, level);
    ASSERT(to < VerticesCount());
    builder_.AddEdge(vertices_.at(level).at(from), to, std::move(edgeProperty));
}

void IntermediateGraph::AddVertex(VertexId vertex, LevelId level) {
    ASSERT(vertex < VerticesCount());
    if (!vertices_.at(level).contains(vertex)) {
        auto id = builder_.AddVertex();
        vertices_.at(level).emplace(vertex, id);
    }
}

void IntermediateGraph::Dump(std::ostream& os) const {
    builder_.graph_.Dump(os);
    topology_.Dump(os);
    ::Dump(os, vertices_);
}

VertexId IntermediateGraph::GetCellId(VertexId vertex, LevelId level) const {
    ASSERT(vertex < VerticesCount());
    return topology_.GetCellId(vertex, level);
}

EdgeProperty IntermediateGraph::GetEdgeProperties(EdgeId edgeId) const {
    return builder_.graph_.GetEdgeProperties(edgeId);
}

VertexId IntermediateGraph::GetTarget(EdgeId edgeId) const {
    return builder_.graph_.GetTarget(edgeId);
}

LevelId IntermediateGraph::LevelsCount() const {
    return topology_.LevelsCount() - 1;
}

void IntermediateGraph::Load(std::istream& is) {
    builder_.graph_.Load(is);
    topology_.Load(is);
    ::Load(is, vertices_);
}

LevelId IntermediateGraph::MaxDistinctLevel(VertexId first, VertexId second) const {
    ASSERT(first < VerticesCount());
    ASSERT(second < VerticesCount());
    return topology_.MaxDistinctLevel(first, second);
}

VertexId IntermediateGraph::VerticesCount() const {
    return vertices_.at(0).size();
}

auto CellInnerTransitions(const IntermediateGraph& graph, VertexId vertex, LevelId level) {
    auto cell = graph.GetCellId(vertex, level);
    auto filter = [=, &graph](EdgeId edgeId) {
        return graph.GetCellId(graph.GetTarget(edgeId), level) == cell;
    };
    const auto& range = graph.GetOutgoingEdges(vertex, level);
    return IteratorRange(
        FilterIterator(range.begin(), range.end(), filter),
        FilterIterator(range.end(), range.end(), filter));
}

CellInnerTransitionsS::CellInnerTransitionsS(VertexId cell, LevelId level)
    : cell_(cell)
    , level_(level)
{}

template <class Function>
void TraverseChildren(
    VertexId vertex,
    const std::unordered_map<VertexId, std::vector<VertexId>>& children,
    const Function& function)
{
    if (!children.contains(vertex)) {
        function(vertex);
        return;
    }
    for (auto child : children.at(vertex)) {
        TraverseChildren(child, children, function);
    }
}

class ThreadLocalDijkstra {
public:
    void Init(const WGraph& originalGraph, const IntermediateGraph& graph, LevelId level) {
        if (!dijkstra_ || level > currentLevel_) {
            if (count_) {
                Log() << "Stats, RunsCount:" << count_ << '\n' << dijkstra_->GetStats() / count_;
                count_ = 0;
            }
            currentLevel_ = level;
            dijkstra_ = std::make_unique<MultilevelDijkstraAlgorithm>(originalGraph);
            dijkstra_->Preprocess([&]() {
                return graph;
            });
        }
    }

    template <class Transitions>
    void Run(VertexId source, Transitions transitions) {
        dijkstra_->FindShortestPathWeight<Finish::ALL_VERTICES, Transitions>(
            source,
            source,  // TODO: remove it, use Finish::ALL in MLD.
            transitions);
        ++count_;
    }

    auto GetDistance(VertexId target) const {
        return dijkstra_->GetShortestDistance(target);
    }

private:
    LevelId currentLevel_ = 0;
    std::unique_ptr<MultilevelDijkstraAlgorithm> dijkstra_;
    int count_ = 0;
};

thread_local ThreadLocalDijkstra threadLocalDijkstra;

IntermediateGraph MultithreadContraction(const WGraph& originalGraph, const Topology& topology) {
    IntermediateGraph graph(originalGraph, topology);

    ASSERT(topology.LevelsCount() > 1);
    const LevelId levels = topology.LevelsCount();
    std::unordered_map<VertexId, std::vector<VertexId>> children;
    std::vector<std::unordered_set<VertexId>> cellsByLevel(levels);
    for (const auto [child, parent] : Enumerate(topology.parents_)) {
        children[parent].push_back(child);
        cellsByLevel.at(topology.Level(parent)).insert(parent);
    }

    const auto numThreads = std::thread::hardware_concurrency();
    ThreadPool threadPool(numThreads);

    auto reversedOriginalGraph = originalGraph.Reversed();

    std::vector<C::LevelStats> stats(levels);

    auto contractCell = [&](
        VertexId cellId,
        LevelId level,
        std::unordered_set<VertexId>& border,
        std::vector<C::Edge>& cut,
        std::vector<C::Edge>& inner)
    {
        ASSERT(border.empty());
        ASSERT(cut.empty());
        ASSERT(inner.empty());

        TraverseChildren(cellId, children, [&](VertexId child) {
            for (auto edgeId : graph.GetOutgoingEdges(child, level - 1)) {
                auto to = graph.GetTarget(edgeId);
                if (cellId != topology.GetCellId(to, level)) {
                    border.insert(child);
                    cut.push_back({child, to, graph.GetEdgeProperties(edgeId).weight});
                }
            }
            if (!border.contains(child)) {
                for (auto edgeId : reversedOriginalGraph.GetOutgoingEdges(child)) {
                    auto to = reversedOriginalGraph.GetTarget(edgeId);
                    if (cellId != topology.GetCellId(to, level)) {
                        border.insert(child);
                        break;
                        // This edge will be added in the other component.
                        // TODO: Remove this for loop for potential speed up.
                    }
                }
            }
        });

        for (auto from : border) {
            threadLocalDijkstra.Init(originalGraph, graph, level);
            threadLocalDijkstra.Run(
                from,
                CellInnerTransitionsS(topology.GetCellId(from, level), level));
            for (auto to : border) {
                if (from != to && threadLocalDijkstra.GetDistance(to) < Dijkstra::INF) {
                    inner.push_back({from, to, threadLocalDijkstra.GetDistance(to)});
                }
            }
        }
    };

    LevelId singleThreadLevel = 0;

    for (LevelId level = singleThreadLevel + 1; level < levels; ++level) {
        Log() << "Contracting level" << static_cast<int>(level) << "...";
        Timer timer;
        auto numCells = cellsByLevel.at(level).size();
        std::unordered_map<VertexId, std::unordered_set<VertexId>> borders(numCells);
        std::unordered_map<VertexId, std::vector<C::Edge>> cutEdges(numCells);
        std::unordered_map<VertexId, std::vector<C::Edge>> innerEdges(numCells);

        for (auto cellId : cellsByLevel.at(level)) {
            borders.try_emplace(cellId);
            cutEdges.try_emplace(cellId);
            innerEdges.try_emplace(cellId);
        }

        std::vector<std::shared_ptr<Task>> tasks(cellsByLevel.at(level).size());
        for (auto [index, cellId] : Enumerate(cellsByLevel.at(level))) {
            tasks[index] = threadPool.AddTask([&, cellId = cellId]() {
                contractCell(cellId, level, borders.at(cellId), cutEdges.at(cellId), innerEdges.at(cellId));
            });
        }

        threadPool.WaitAll();
        for (auto& task : tasks) {
            bool completed = task->IsCompletedOrThrow();
            ASSERT(completed);
        }

        for (auto cellId : cellsByLevel.at(level)) {
            for (auto [from, to, weight] : cutEdges.at(cellId)) {
                graph.AddEdge(from, to, level, EdgeProperty{weight});
            }
            for (auto [from, to, weight] : innerEdges.at(cellId)) {
                graph.AddEdge(from, to, level, EdgeProperty{weight});
            }

            stats.at(level) += C::LevelStats{
                C::LevelStats::CellStats{
                    1,
                    static_cast<VertexId>(children.at(cellId).size()),
                    static_cast<VertexId>(children.at(cellId).size()),
                },
                static_cast<EdgeId>(cutEdges.at(cellId).size()),
                static_cast<EdgeId>(innerEdges.at(cellId).size()),
            };
        }

        Log() << "level" << static_cast<int>(level) << "contracted in"
              << timer.Elapsed() / 1'000'000 << "ms";
        Log() << "vertices:" << graph.vertices_.at(level).size();
    }

    stats.at(0) = C::LevelStats{
        C::LevelStats::CellStats{
            originalGraph.VerticesCount(),
            1,
            1,
        },
        originalGraph.EdgesCount(),
        0,
    };

    Log() << stats;
    graph.stats_ = std::move(stats);

    return graph;
}

std::ostream& operator<<(std::ostream& os, const C::LevelStats::CellStats& stats) {
    os << "count " << stats.count << ", ";
    os << "minSize " << stats.minSize << ", ";
    os << "maxSize " << stats.maxSize;
    return os;
}

std::ostream& operator<<(std::ostream& os, const C::LevelStats& stats) {
    os << "cellStats: " << stats.cellStats << "; ";
    os << "cutEdges " << stats.cutEdges << "; ";
    os << "innerEdges " << stats.innerEdges << std::endl;
    return os;
}
