#include "graph.h"
#include "utils.h"

template <class G, class Predicate>
class FilteredGraph {
public:
    FilteredGraph(const G& graph, Predicate predicate)
        : graph_(graph)
        , predicate_(std::move(predicate))
    {}

    auto GetOutgoingEdges(VertexId vertex) const {
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

struct FlowProperties {
    int capacity = 0;
    int flow = 0;

    bool operator==(const FlowProperties& rhs) const = default;
};

class FlowNetwork : public WeightGraph<FlowProperties> {
public:
    void AddFlow(EdgeId edgeId, int flow);

    int GetFlow() const;

    VertexId GetFlowSource() const;

    VertexId GetSink() const;

    int ResidualCapacity(EdgeId edgeId) const;

    auto ResidualNetworkView() const {
        auto predicate = [this](EdgeId edgeId) {
            return ResidualCapacity(edgeId) > 0;
        };

        return FilteredGraph(*this, predicate);
    }

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
