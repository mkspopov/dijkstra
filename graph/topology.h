#pragma once

#include "types.h"
#include "utils.h"

#include <vector>

struct Topology {
    Topology() = default;

    void Dump(std::ostream& os) const;
    void Load(std::istream& is);

    VertexId GetCellId(VertexId vertexId) const;

    VertexId GetCellId(VertexId vertexId, LevelId level) const;

    LevelId Level(VertexId from) const;

    LevelId LevelsCount() const;

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const;

    std::vector<VertexId> parents_;
    std::vector<VertexId> sizes_;
};
