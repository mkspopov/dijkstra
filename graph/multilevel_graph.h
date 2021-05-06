#pragma once

#include "graph.h"

#include <cassert>
#include <iterator>
#include <unordered_set>

struct Topology {
    std::vector<std::vector<VertexId>> cells_;
    std::vector<VertexId> sizes_;
};

class MultilevelGraph {
public:
    explicit MultilevelGraph(const Graph& graph, Topology topology)
        : graph_(graph), topology_(std::move(topology)), cells_(topology_.cells_),
          sizes_(topology_.sizes_) {
        assert(!sizes_.empty());
        assert(sizes_.front() == 0);

        GraphBuilder builder;
        for (const auto& layer : cells_) {
            std::unordered_set<VertexId> visitedVertices;
            for (auto vertex : layer) {
                if (!visitedVertices.contains(vertex)) {
                    builder.AddVertex();
                    visitedVertices.insert(vertex);
                }
            }
        }

        // for each vertex we have component edges and cut edges
        for (LevelId level = 0; level < LevelsCount(); ++level) {
            for (const auto& [edgeId, edge] : Enumerate(graph_.GetEdges())) {
                auto cellFrom = Cell(edge.from, level);
                auto cellTo = Cell(edge.to, level);
                if (cellFrom != cellTo) {
                    builder.AddEdge(cellFrom, cellTo, graph_.GetEdgeProperties(edgeId));
                }
            }
        }

        multilevelGraph_ = builder.Build();
    }

    EdgeProperty GetEdgeProperties(EdgeId edgeId) const;

//    const auto& GetOutgoingEdges(VertexId from) const {
//        return edges_[from];
//    }

    VertexId GetTarget(EdgeId edgeId) const {
        return graph_.GetTarget(edgeId);
    }

    VertexId VerticesCount() const {
        return graph_.VerticesCount();
    }

    VertexId VerticesCount(LevelId level) const {
        if (level == 0) {
            return VerticesCount();
        }
        return sizes_[level] - sizes_[level - 1];
    }

//    EdgeId EdgesCount() const {
//        return edges_.size();
//    }

    Graph Reversed() const;

    LevelId Level(VertexId from) const {
        return std::distance(sizes_.begin(), std::upper_bound(sizes_.begin(), sizes_.end(), from));
    }

    LevelId MaxDistinctLevel(VertexId first, VertexId second) const {
        assert(cells_.size() > 1);
        LevelId level = 1;
        while (cells_[level][first] != cells_[level][second]) {
            ++level;
        }
        return level - 1;
    }

    VertexId Cell(VertexId vertex, LevelId level) const {
        return cells_[level][vertex] + sizes_[level];
    }

    LevelId LevelsCount() const {
        return cells_.size();
    }

private:
    const Graph& graph_;
    Topology topology_;

    std::vector<std::vector<VertexId>> cells_;
    std::vector<VertexId> sizes_;

    Graph multilevelGraph_;
};
