#include "graph.h"
#include "utils.h"

template <class G, class Predicate>
class FilteredGraph {
public:
    FilteredGraph(const G& graph, Predicate predicate);

    auto GetOutEdgeIds(VertexId vertex) const;

    VertexId Source(EdgeId edge_id) const;

    VertexId Target(EdgeId edge_id) const;

    VertexId VerticesNumber() const;

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
        std::vector<EdgeId>* path);

    void DiscoverVertex(VertexId vertex);

    void TreeEdge(EdgeId edge_id);

private:
    const GraphType& graph_;
    const VertexId source_ = 0;
    const VertexId target_ = 0;
    std::vector<EdgeId> parent_;
    std::vector<EdgeId>* path_ = nullptr;
};

class FlowNetwork {
public:
    void AddFlow(EdgeId edge_id, int flow);

    int GetFlow() const;

    int GetMaxPossibleFlow() const;

    VertexId GetSource() const;

    VertexId GetSink() const;

    int ResidualCapacity(EdgeId edge_id) const;

    auto ResidualNetworkView() const;

private:
    friend class FlowNetworkBuilder;

    EdgeId BackwardEdgeId(EdgeId edge_id) const;

    struct EdgeProperties : public ::EdgeProperty {
        explicit EdgeProperties(int capacity);

        int capacity = 0;
        int flow = 0;
    };

    std::vector<EdgeProperties> edge_properties_;
    Graph graph_;
    VertexId source_ = 0;
    VertexId sink_ = 0;
};

class FlowNetworkBuilder {
public:
    void AddEdgeWithCapacity(const Edge& edge, int capacity);

    FlowNetwork&& Build();

    void SetSink(VertexId sink);

    void SetSource(VertexId source);

    void SetVerticesNumber(VertexId vertices_number);

private:
    FlowNetwork network_;
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
    std::vector<Color> colors(graph.VerticesNumber(), Color::WHITE);
    std::queue<VertexId> queue;

    queue.push(source);
    colors[source] = Color::BLACK;

    while (!queue.empty()) {
        auto vertex = queue.front();
        queue.pop();
        for (auto edge_id : graph.GetOutEdgeIds(vertex)) {
            auto target = graph.Target(edge_id);
            if (colors[target] == Color::WHITE) {
                visitor.TreeEdge(edge_id);
                colors[target] = Color::BLACK;
                visitor.DiscoverVertex(target);
                queue.push(target);
            }
        }
    }
}
