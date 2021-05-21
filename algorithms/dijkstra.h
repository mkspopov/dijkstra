#pragma once

#include "color.h"
#include "graph.h"
#include "heap.h"
#include "visitor.h"

#include <algorithm>
#include <limits>
#include <unordered_set>
#include <vector>

class Dijkstra {
public:
    struct Stats {
        VertexId verticesDiscovered = 0;
        VertexId verticesExamined = 0;
        EdgeId edgesExamined = 0;
        EdgeId edgesRelaxed = 0;
        EdgeId edgesNotRelaxed = 0;
    };

    static inline constexpr Weight INF = std::numeric_limits<Weight>::infinity();
    static inline constexpr Weight START_WEIGHT = 0;
    static inline constexpr VertexId UNDEFINED_VERTEX = std::numeric_limits<VertexId>::max();

    explicit Dijkstra(const WGraph& graph);

    auto AffectedVertices() {
        return IteratorRange(affectedVertices_.begin(), affectedVertices_.end());
    }

    static constexpr std::string_view GetName() {
        return "Dijkstra";
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

    // Visitor api
    void DiscoverVertex(VertexId vertex);
    void ExamineEdge(EdgeId);
    void ExamineVertex(VertexId vertex);
    void EdgeNotRelaxed(EdgeId);
    void EdgeRelaxed(EdgeId);
    void FinishVertex(VertexId vertex);

    Stats GetStats() const {
        return stats_;
    }

protected:
    void Clear();

    void ClearHeap();

    const WGraph& graph_;
    std::vector<Weight> distances_;
    std::vector<VertexId> affectedVertices_;
    StdHeap heap_;
    std::vector<Color> colors_;

    std::unordered_set<VertexId> targets_;

    Stats stats_;
};

std::ostream& operator<<(std::ostream& os, const Dijkstra::Stats& s);

Dijkstra::Stats& operator+=(Dijkstra::Stats& lhs, const Dijkstra::Stats& rhs);
Dijkstra::Stats operator/(Dijkstra::Stats lhs, int rhs);

template <class Queue, class G, class Visitor>
void BoostDijkstra(
    const G& graph,
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

template <class Queue, class G, class Visitor>
auto BoostDijkstra(
    const G& graph,
    const std::vector<VertexId>& sources,
    Visitor& visitor)
{
    std::vector<Weight> distances(graph.VerticesCount(), Dijkstra::INF);
    std::vector<Color> colors(graph.VerticesCount(), Color::WHITE);
    Queue queue;
    BoostDijkstra(graph, distances, colors, sources, queue, visitor);
    return distances;
}
