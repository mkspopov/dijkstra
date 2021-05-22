#include "topology.h"

#include <iostream>

void Topology::Dump(std::ostream& os) const {
    ::Dump(os, parents_);
    ::Dump(os, sizes_);
}

VertexId Topology::GetCellId(VertexId vertexId) const {
    return parents_.at(vertexId);
}

VertexId Topology::GetCellId(VertexId vertexId, LevelId level) const {
    ASSERT(vertexId < sizes_.at(1));
    for (LevelId cur = 0; cur < level; ++cur) {
        vertexId = parents_.at(vertexId);
    }
    return vertexId;
}

LevelId Topology::Level(VertexId from) const {
    return std::distance(sizes_.begin(), std::upper_bound(sizes_.begin(), sizes_.end(), from)) - 1;
}

LevelId Topology::LevelsCount() const {
    return sizes_.size();
}

void Topology::Load(std::istream& is) {
    ::Load(is, parents_);
    ::Load(is, sizes_);
}

LevelId Topology::MaxDistinctLevel(VertexId first, VertexId second) const {
    if (first == second) {
        return 0;
    }

    ASSERT(first < sizes_.at(1));
    ASSERT(second < sizes_.at(1));
    LevelId level = 0;
    while (parents_.at(first) != parents_.at(second)) {
        first = parents_.at(first);
        second = parents_.at(second);
        ++level;
    }
    return level;
}
