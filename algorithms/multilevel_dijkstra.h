#pragma once

#include "contraction.h"
#include "dijkstra.h"
#include "multilevel_graph.h"
#include "visitor.h"

/*
 * MultiLevelGraph is not used to calculate single-to-all shortest paths.
 * That is, source and target vertices are known from the beginning.
 */

template <class G>
LevelId Level(
    const G& graph,
    VertexId source,
    VertexId target,
    VertexId vertex)
{
    return std::min(graph.MaxDistinctLevel(source, vertex), graph.MaxDistinctLevel(target, vertex));
}

enum class Finish {
    ALL_VERTICES,
    ALL_TARGETS,
    FIRST_TARGET,
};

template <Finish FinishStrategy = Finish::ALL_VERTICES, class Queue, class G, class Transitions, class Visitor>
void MultilevelDijkstra(
    const G& graph,
    std::vector<Weight>& distances,
    std::vector<Color>& colors,
    const std::vector<VertexId>& sources,
    const std::unordered_set<VertexId>& targets,
    Transitions transitions,
    Queue& queue,
    Visitor& visitor)
{
    // TODO: Deal with many targets.
    ASSERT_EQUAL(targets.size(), 1ul);

    for (auto source : sources) {
        visitor.DiscoverVertex(source);
        distances.at(source) = 0;
        colors.at(source) = Color::GRAY;
        queue.Emplace(source, 0);
    }

    while (!queue.Empty()) {
        auto [from, dist] = queue.Extract();
        // TODO: Optional check to easily deal with
        // std::priority_queue. This may slow down
        // code because of memory load of a random vertex
        // of the graph. So, measure time without it.
        if (colors.at(from) == Color::BLACK) {
            continue;
        }

        visitor.ExamineVertex(from);

        if constexpr (FinishStrategy == Finish::FIRST_TARGET) {
            if (targets.contains(from)) {
                return;
            }
        } else if constexpr (FinishStrategy == Finish::ALL_TARGETS) {
            // TODO:
        }

        auto level = Level(graph, sources.at(0), *targets.begin(), from);  // deal with many targets?

        for (auto edgeId : transitions(graph, from, level)) {
            visitor.ExamineEdge(edgeId);
            auto to = graph.GetTarget(edgeId);
            auto relaxedDist = dist + graph.GetEdgeProperties(edgeId).weight;
            if (relaxedDist < distances.at(to)) {
                distances.at(to) = relaxedDist;
                if (colors.at(to) == Color::WHITE) {
                    visitor.DiscoverVertex(to);
                    colors.at(to) = Color::GRAY;
                    queue.Emplace(to, relaxedDist);
                } else if (colors.at(to) == Color::GRAY) {
                    queue.Decrease({to, relaxedDist});
                }
                visitor.EdgeRelaxed(edgeId);
            } else {
                visitor.EdgeNotRelaxed(edgeId);
            }
        }
        colors.at(from) = Color::BLACK;
        visitor.FinishVertex(from);
    }
}

template <class Queue, class G, class Transitions, class Visitor>
auto MultilevelDijkstra(
    const G& graph,
    const std::vector<VertexId>& sources,
    const std::vector<VertexId>& targets,
    Transitions transitions,
    Visitor& visitor)
{
    std::vector<Weight> distances(graph.VerticesCount(), Dijkstra::INF);
    std::vector<Color> colors(graph.VerticesCount(), Color::WHITE);
    Queue queue;
    MultilevelDijkstra(graph, distances, colors, sources,
        ContainerCast<std::unordered_set<VertexId>>(targets), std::move(transitions), queue, visitor);
    return distances;
}

struct AllTransitions {
    template <class G>
    auto operator()(const G& graph, VertexId from, LevelId level) const {
        return graph.GetOutgoingEdges(from, level);
    }
};

template <class Queue, class G>
auto MultilevelDijkstra(
    const G& graph,
    const std::vector<VertexId>& sources,
    const std::vector<VertexId>& targets)
{
    return MultilevelDijkstra<Queue>(graph, sources, targets, AllTransitions{}, GetTrivialVisitor());
}

class MultilevelDijkstraAlgorithm : public Dijkstra {
public:
    explicit MultilevelDijkstraAlgorithm(const WGraph& graph);

    template <Finish FINISH_CRITERION = Finish::FIRST_TARGET, class Transitions = AllTransitions>
    Weight FindShortestPathWeight(VertexId source, VertexId target, Transitions transitions = {}) {
        InitSearch(source);
        auto& dijkstraVisitor = *this;
        MultilevelDijkstra<FINISH_CRITERION>(
            graph_, distances_, colors_, {source}, {target}, transitions, heap_, dijkstraVisitor);
        return distances_[target];
    }

    LevelId LevelsCount() const;

    void Preprocess(const std::function<IntermediateGraph()>& preprocessor);

private:
    const WGraph& GetOriginalGraph() const;

    IntermediateGraph graph_;
};

IntermediateGraph PreprocessGraph(
    const WGraph& originalGraph,
    const std::filesystem::path& path,
    LevelId levels);
