#include "contraction.h"

#include <iostream>
#include <vector>

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
