#pragma once

#include "color.h"
#include "types.h"

#include <queue>

template <class GraphType, class Visitor>
void TraverseGraphInBfsOrder(
    const GraphType& graph,
    VertexId source,
    Visitor visitor)
{
    std::vector<Color> colors(graph.VerticesCount(), Color::WHITE);
    std::queue<VertexId> queue;

    queue.push(source);
    colors[source] = Color::BLACK;

    while (!queue.empty()) {
        auto vertex = queue.front();
        queue.pop();
        for (auto edgeId : graph.GetOutgoingEdges(vertex)) {
            auto target = graph.GetTarget(edgeId);
            if (colors[target] == Color::WHITE) {
                visitor.TreeEdge(edgeId);
                colors[target] = Color::BLACK;
                visitor.DiscoverVertex(target);
                queue.push(target);
            }
        }
    }
}
