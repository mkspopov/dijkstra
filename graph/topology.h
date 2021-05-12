#pragma once

#include "types.h"
#include "utils.h"

#include <vector>

struct Topology {
    VertexId GetCellId(VertexId vertexId) const {
        return parents_.at(vertexId);
    }

    VertexId GetCellId(VertexId vertexId, LevelId level) const {
        auto childOnZeroLevel = vertexId;
        while (childOnZeroLevel >= sizes_.at(1)) {
            childOnZeroLevel = children_.at(childOnZeroLevel).front();
        }
        return cells_.at(level).at(childOnZeroLevel) + sizes_.at(level);
    }

    LevelId Level(VertexId from) const {
        return std::distance(sizes_.begin(), std::upper_bound(sizes_.begin(), sizes_.end(), from));
    }

    LevelId LevelsCount() const {
        return sizes_.size();
    }

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const {
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

    std::vector<std::vector<VertexId>> cells_;
    std::vector<VertexId> sizes_;

    std::vector<std::vector<VertexId>> cellIds_;
    std::vector<std::vector<VertexId>> children_;
    std::vector<VertexId> parents_;
};

struct CompactTopology {
    VertexId GetCellId(VertexId vertexId) const {
        return parents_.at(vertexId);
    }

    VertexId GetCellId(VertexId vertexId, LevelId level) const {
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

    LevelId Level(VertexId from) const {
        return std::distance(sizes_.begin(), std::upper_bound(sizes_.begin(), sizes_.end(), from));
    }

    LevelId LevelsCount() const {
        return sizes_.size();
    }

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const {
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

    std::vector<VertexId> parents_;
    std::vector<VertexId> sizes_;
};
