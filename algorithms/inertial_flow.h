#pragma once

#include "min_cut.h"
#include "thread_pool.h"

#include <algorithm>
#include <execution>

struct State {

    State(VertexId verticesCount) : toLocalId(verticesCount), toGraphId(verticesCount) {
    }

    std::vector<VertexId> toLocalId;
    std::vector<VertexId> toGraphId;
};

struct MappingToOriginalGraph {
    CoordGraph graph;
    std::vector<VertexId> mapping;
};

template <class Comp>
auto MakePartition(const CoordGraph& originalGraph, LevelId levels, double coef, int steps) {
    (void) steps;
    auto iota = Range(0u, originalGraph.VerticesCount());
    std::vector<MappingToOriginalGraph> graphs{{originalGraph, std::vector(iota.begin(), iota.end())}};
    std::vector<State> states;

    ThreadPool threadPool(std::thread::hardware_concurrency());

    for (LevelId level = 0; level < levels; ++level) {
//        for (int step = 0; step < steps; ++step) {
        std::vector<std::unordered_set<VertexId>> parts(graphs.size() * 2);

        auto split = [&](size_t index) {
            const auto& toSplit = graphs[index];

            auto vertices = ContainerCast<std::vector<VertexId>>(
                Range(0u, toSplit.graph.VerticesCount()));

            // TODO: Sort once in the beginning.
            std::sort(vertices.begin(), vertices.end(), Comp(toSplit.graph));
            auto minCut = FindMinCut(
                std::vector(vertices.begin(), vertices.begin() + coef * vertices.size()),
                std::vector(vertices.begin() + (1 - coef) * vertices.size(), vertices.end()),
                toSplit.graph);

            auto secondVertices = ContainerCast<std::unordered_set<VertexId>>(vertices);
            for (auto v : minCut.first) {
                ASSERT_EQUAL(secondVertices.erase(v), 1ul);
            }

            for (auto vertex : minCut.first) {
                parts[2 * index].insert(toSplit.mapping[vertex]);
            }
            for (auto vertex : secondVertices) {
                parts[2 * index + 1].insert(toSplit.mapping[vertex]);
            }
        };

        std::vector<std::shared_ptr<Task>> tasks(graphs.size());
        for (auto[i, _] : Enumerate(graphs)) {
            tasks[i] = threadPool.AddTask([&, i = i]() {
                split(i);
            });
        }

        threadPool.WaitAll();
        for (auto& task : tasks) {
            ASSERT(task->IsCompletedOrThrow());
        }

        auto last = std::remove_if(
            std::execution::par,
            parts.begin(), parts.end(),
            [](const auto& part) {
                return part.empty();
            });
        parts.erase(last, parts.end());

        tasks.resize(parts.size());
        State state(originalGraph.VerticesCount());
        for (const auto&[i, part] : Enumerate(parts)) {
            tasks[i] = threadPool.AddTask([&, i = i]() {
                for (auto[innerIndex, v] : Enumerate(parts[i])) {
                    state.toGraphId[v] = i;
                    state.toLocalId[v] = innerIndex;
                }
            });
        }
        threadPool.WaitAll();
        for (auto& task : tasks) {
            ASSERT(task->IsCompletedOrThrow());
        }

        std::vector<MappingToOriginalGraph> partedGraphs(parts.size());
        for (const auto&[i, _] : Enumerate(parts)) {
            tasks[i] = threadPool.AddTask([&, graphInd = i]() {
                CoordGraph& builder = partedGraphs[graphInd].graph;
                auto& mapping = partedGraphs[graphInd].mapping;
                for (auto [innerIndex, v] : Enumerate(parts[graphInd])) {
                    builder.AddVertex(originalGraph.GetVertexProperties(v));
                    mapping.push_back(v);
                }
                for (auto[innerIndex, v] : Enumerate(parts[graphInd])) {
                    for (auto edgeId : originalGraph.GetOutgoingEdges(v)) {
                        auto to = originalGraph.GetTarget(edgeId);
                        if (state.toGraphId[to] == graphInd) {
                            builder.AddEdge(state.toLocalId[v], state.toLocalId[to]);
                        }
                    }
                }
            });
        }
        threadPool.WaitAll();
        for (auto& task : tasks) {
            ASSERT(task->IsCompletedOrThrow());
        }

        graphs = std::move(partedGraphs);
        states.push_back(std::move(state));
//        }
    }
    return states;
}
