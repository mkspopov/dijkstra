#include "color.h"
#include "graph.h"
#include "utils.h"

#include <queue>

template <class G, class Predicate>
class FilteredGraph {
public:
    FilteredGraph(const G& graph, Predicate predicate)
        : graph_(graph)
        , predicate_(std::move(predicate))
    {}

    auto GetOutEdgeIds(VertexId vertex) const {
        auto edgesRange = graph_.GetOutgoingEdges(vertex);
        return IteratorRange(
            FilterIterator(edgesRange.begin(), edgesRange.end(), predicate_),
            FilterIterator(edgesRange.end(), edgesRange.end(), predicate_));
    }

    VertexId GetSource(EdgeId edgeId) const {
        return graph_.GetSource(edgeId);
    }

    VertexId GetTarget(EdgeId edgeId) const {
        return graph_.GetTarget(edgeId);
    }

    VertexId VerticesCount() const {
        return graph_.VerticesCount();
    }

private:
    const G& graph_;
    Predicate predicate_;
};

template <class GraphType>
class ShortestPathBfsVisitor {
public:
    ShortestPathBfsVisitor(
        const GraphType& graph,
        VertexId source,
        VertexId target,
        std::vector<EdgeId>* path)
            : graph_(graph)
            , source_(source)
            , target_(target)
            , parent_(graph_.VerticesCount())
            , path_(path)
    {}

    void DiscoverVertex(VertexId vertex) {
        if (vertex == target_) {
            while (vertex != source_) {
                auto parentEdgeId = parent_[vertex];
                path_->push_back(parentEdgeId);
                vertex = graph_.GetSource(parentEdgeId);
            }
            std::reverse(path_->begin(), path_->end());
        }
    }

    void TreeEdge(EdgeId edgeId) {
        parent_[graph_.GetTarget(edgeId)] = edgeId;
    }

private:
    const GraphType& graph_;
    const VertexId source_ = 0;
    const VertexId target_ = 0;
    std::vector<EdgeId> parent_;
    std::vector<EdgeId>* path_ = nullptr;
};

struct FlowProperties {
    EdgeId id;
    int capacity = 0;
    int flow = 0;

    bool operator==(const FlowProperties& rhs) const = default;
};

class FlowNetwork : public WeightGraph<FlowProperties> {
public:
    void AddFlow(EdgeId edgeId, int flow);

    int GetFlow() const;

    int GetMaxPossibleFlow() const;

    VertexId GetFlowSource() const;

    VertexId GetSink() const;

    int ResidualCapacity(EdgeId edgeId) const;

    auto ResidualNetworkView() const;

private:
    friend class FlowNetworkBuilder;

    EdgeId BackwardEdgeId(EdgeId edgeId) const;

    VertexId source_ = 0;
    VertexId sink_ = 0;
};

class FlowNetworkBuilder {
public:
    FlowNetworkBuilder() = default;

    void AddEdgeWithCapacity(const Edge& edge, int capacity);

    FlowNetwork&& Build();

    void SetSink(VertexId sink);

    void SetSource(VertexId source);

    void SetVerticesCount(VertexId verticesCount);

private:
    GraphBuilder<FlowNetwork, FlowProperties> builder_;
};

class EdmondsKarp {
public:
    explicit EdmondsKarp(FlowNetwork* network);

    int FindMaxFlow();

private:
    void UpdateFlow(const std::vector<EdgeId>& path);

    FlowNetwork* network_ = nullptr;
};

template <class Predicate>
int BinarySearch(int left, int right, Predicate predicate) {
    while (left < right) {
        auto mid = left + (right - left) / 2;
        if (predicate(mid)) {
            right = mid;
        } else {
            left = mid + 1;
        }
    }
    if (predicate(left)) {
        return left;
    }
    return right;
}

template <class GraphType, class Visitor>
void TraverseGraphInBfsOrder(
    const GraphType& graph,
    VertexId source,
    Visitor visitor) {
    std::vector<Color> colors(graph.VerticesCount(), Color::WHITE);
    std::queue<VertexId> queue;

    queue.push(source);
    colors[source] = Color::BLACK;

    while (!queue.empty()) {
        auto vertex = queue.front();
        queue.pop();
        for (auto edgeId : graph.GetOutEdgeIds(vertex)) {
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
