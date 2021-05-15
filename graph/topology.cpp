#include "topology.h"

#include <iostream>

VertexId Topology::GetCellId(VertexId vertexId) const {
    return parents_.at(vertexId);
}

VertexId Topology::GetCellId(VertexId vertexId, LevelId level) const {
    auto childOnZeroLevel = vertexId;
    while (childOnZeroLevel >= sizes_.at(1)) {
        childOnZeroLevel = children_.at(childOnZeroLevel).front();
    }
    return cells_.at(level).at(childOnZeroLevel) + sizes_.at(level);
}

LevelId Topology::Level(VertexId from) const {
    return std::distance(sizes_.begin(), std::upper_bound(sizes_.begin(), sizes_.end(), from));
}

LevelId Topology::LevelsCount() const {
    return sizes_.size();
}

LevelId Topology::MaxDistinctLevel(VertexId first, VertexId second) const {
    auto firstLevel = Level(first);
    auto secondLevel = Level(second);

    while (firstLevel < secondLevel) {
        first = parents_.at(first);
        ++firstLevel;
    }

    while (secondLevel < firstLevel) {
        second = parents_.at(second);
        ++secondLevel;
    }

    if (first == second) {
        return 0;
    }

    ASSERT(cells_.size() > 1);
    while (parents_.at(first) != parents_.at(second)) {
        first = parents_.at(first);
        second = parents_.at(second);
        ++firstLevel;
    }
    return firstLevel - 1;
}

void CompactTopology::Dump(std::ostream& os) const {
    ::Dump(os, parents_);
    ::Dump(os, sizes_);
}

VertexId CompactTopology::GetCellId(VertexId vertexId) const {
    return parents_.at(vertexId);
}

VertexId CompactTopology::GetCellId(VertexId vertexId, LevelId level) const {
    if (level == 0) {
        ASSERT(vertexId < sizes_.at(1));
        return vertexId;
        // Do we really need it? May be ASSERT(level > 0)?
    }
    ASSERT(vertexId < sizes_.at(1));
    for (LevelId cur = 0; cur < level; ++cur) {
        vertexId = parents_.at(vertexId);
    }
    return vertexId;
}

LevelId CompactTopology::Level(VertexId from) const {
    return std::distance(sizes_.begin(), std::upper_bound(sizes_.begin(), sizes_.end(), from));
}

LevelId CompactTopology::LevelsCount() const {
    return sizes_.size();
}

void CompactTopology::Load(std::istream& is) {
    ::Load(is, parents_);
    ::Load(is, sizes_);
}

LevelId CompactTopology::MaxDistinctLevel(VertexId first, VertexId second) const {
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
