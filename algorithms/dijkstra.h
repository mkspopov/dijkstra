#pragma once

#include "heap.h"
#include "graph.h"
#include "visitor.h"

#include <algorithm>
#include <limits>
#include <unordered_set>
#include <vector>

enum class Color {
    WHITE,
    GRAY,
    BLACK,
};

class Dijkstra {
public:
    static inline constexpr Weight INF = std::numeric_limits<Weight>::infinity();
    static inline constexpr Weight START_WEIGHT = 0;
    static inline constexpr VertexId UNDEFINED_VERTEX = std::numeric_limits<VertexId>::max();

    explicit Dijkstra(const Graph& graph);

    auto AffectedVertices() {
        return IteratorRange(affectedVertices_.begin(), affectedVertices_.end());
    }

    bool IsAffected(VertexId vertexId) const;

    bool IsProcessed(VertexId vertexId) const;

    void Preprocess();

    Weight FindShortestPathWeight(VertexId source, VertexId target);

    void FindShortestPathsWeights(VertexId source);

    VertexId ProcessVertex();

    void ProcessVertex(VertexId from, float fromDistance);

    /*
     * Set targets to prevent Dijkstra from unnecessary running.
     */
    template <class Container>
    void SetTargets(const Container& targets) {
        ASSERT(targets_.empty());
        for (const auto target : targets) {
            targets_.insert(target);
        }
    }

    /*
     * Get the shortest distance after FindShortestPathsWeights.
     */
    Weight GetShortestDistance(VertexId target) const;

    void InitSearch(VertexId source);

protected:
    void NotRelaxed(EdgeId);

    void RelaxEdge(EdgeId);

    void ExamineEdge(EdgeId);

    void ExamineVertex(VertexId vertex);

//    bool IsVisited(VertexId vertex) const {
////        return distances_[vertex] != INF;
//        return colors_[vertex] != Color::WHITE;
//    }

    void DiscoverVertex(VertexId vertex);

    void Clear();

    void ClearHeap();

    const Graph& graph_;
    std::vector<Weight> distances_;
    std::vector<VertexId> affectedVertices_;
    StdHeap heap_;
    std::vector<Color> colors_;

    std::unordered_set<VertexId> targets_;
};

template <class Queue, class Visitor>
void BoostDijkstra(
    const Graph& graph,
    std::vector<Weight>& distances,
    std::vector<Color>& colors,
    const std::vector<VertexId>& sources,
    Queue& queue,
    Visitor& visitor)
{
    for (auto source : sources) {
        distances[source] = 0;
        colors[source] = Color::GRAY;
        queue.Emplace(source, 0);
        visitor.SourceVertex(source);
    }
    
    while (!queue.Empty()) {
        auto [from, dist] = queue.Extract();
        // TODO: Optional check to easily deal with 
        // std::priority_queue. This may slow down
        // code because of memory load of a random vertex
        // of the graph. So, measure time without it.
        if (colors[from] == Color::BLACK) {
            continue;
        }
        visitor.ExamineVertex(from);

        for (auto edgeId : graph.GetOutgoingEdges(from)) {
            visitor.ExamineEdge(edgeId);
            auto to = graph.GetTarget(edgeId);
            auto relaxedDist = dist + graph.GetEdgeProperties(edgeId).weight;
            if (relaxedDist < distances[to]) {
                distances[to] = relaxedDist;
                colors[to] = Color::GRAY;
                if (colors[to] == Color::WHITE) {
                    queue.Emplace(to, relaxedDist);
                } else if (colors[to] == Color::GRAY) {
                    queue.Decrease(to, relaxedDist);
                }
                visitor.EdgeRelaxed(edgeId);
            }
        }
        colors[from] = Color::BLACK;
        visitor.FinishVertex(from);
    }
}

template <class Queue, class Visitor>
auto BoostDijkstra(
    const Graph& graph,
    const std::vector<VertexId>& sources,
    Visitor& visitor)
{
    std::vector<Weight> distances(graph.VerticesCount(), Dijkstra::INF);
    std::vector<Color> colors(graph.VerticesCount(), Color::WHITE);
    Queue queue;
    BoostDijkstra(graph, distances, colors, sources, queue, visitor);
    return distances;
}
