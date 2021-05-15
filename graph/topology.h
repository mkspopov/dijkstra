#pragma once

#include "types.h"
#include "utils.h"

#include <vector>

struct Topology {
    VertexId GetCellId(VertexId vertexId) const;

    VertexId GetCellId(VertexId vertexId, LevelId level) const;

    LevelId Level(VertexId from) const;

    LevelId LevelsCount() const;

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const;

    std::vector<std::vector<VertexId>> cells_;
    std::vector<VertexId> sizes_;

    std::vector<std::vector<VertexId>> cellIds_;
    std::vector<std::vector<VertexId>> children_;
    std::vector<VertexId> parents_;
};

struct CompactTopology {
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
