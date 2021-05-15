#pragma once

#include "contraction.h"
#include "dijkstra.h"
#include "multilevel_graph.h"

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

template <class G, class Transitions, class Queue>
void MultilevelDijkstra(
    const G& graph,
    std::vector<Weight>& distances,
    std::vector<Color>& colors,
    const std::vector<VertexId>& sources,
    const std::vector<VertexId>& targets,
    Transitions transitions,
    Queue& queue)
{
    // TODO: Deal with many targets.
    ASSERT_EQUAL(targets.size(), 1ul);

    for (auto source : sources) {
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

        auto level = Level(graph, sources.at(0), targets.at(0), from);  // deal with many targets?
//        auto cell = graph.GetCellId(from, level);

        for (auto edgeId : transitions(graph, from, level)) {
            auto to = graph.GetTarget(edgeId);
            auto relaxedDist = dist + graph.GetEdgeProperties(edgeId).weight;
            if (relaxedDist < distances.at(to)) {
                distances.at(to) = relaxedDist;
                if (colors.at(to) == Color::WHITE) {
                    colors.at(to) = Color::GRAY;
                    queue.Emplace(to, relaxedDist);
                } else if (colors.at(to) == Color::GRAY) {
                    queue.Decrease({to, relaxedDist});
                }
            }
        }
        colors.at(from) = Color::BLACK;
    }
}

template <class Queue, class G, class Transitions>
auto MultilevelDijkstra(
    const G& graph,
    const std::vector<VertexId>& sources,
    const std::vector<VertexId>& targets,
    Transitions transitions)
{
    std::vector<Weight> distances(graph.VerticesCount(), Dijkstra::INF);
    std::vector<Color> colors(graph.VerticesCount(), Color::WHITE);
    Queue queue;
    MultilevelDijkstra(graph, distances, colors, sources, targets, std::move(transitions), queue);
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
    return MultilevelDijkstra<Queue>(graph, sources, targets, AllTransitions{});
}

class MultilevelDijkstraAlgorithm : public Dijkstra {
public:
    explicit MultilevelDijkstraAlgorithm(const Graph& graph);

    Weight FindShortestPathWeight(VertexId source, VertexId target);

    void Preprocess(std::filesystem::path path);

private:
    const Graph& GetOriginalGraph() const;

    IntermediateGraph graph_;
};
