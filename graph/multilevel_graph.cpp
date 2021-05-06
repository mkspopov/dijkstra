#include "multilevel_graph.h"

#include <iostream>
#include <vector>

EdgeProperty MultilevelGraph::GetEdgeProperties(EdgeId edgeId) const {
    return graph_.GetEdgeProperties(edgeId);
}
