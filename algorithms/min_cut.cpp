#include <algorithm>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <tuple>
#include <vector>

void PrintAnswer(int min_max_gold_bars);

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);

    auto [people_number, gold_bars, trust_relations] = ReadInput();

    auto vertices_number = people_number + 2;
    int source = 0;
    auto sink = vertices_number - 1;

    auto min_max_gold_bars = FindMinMaxGoldBars(
        people_number,
        vertices_number,
        source,
        sink,
        gold_bars,
        trust_relations);

    PrintAnswer(min_max_gold_bars);
}

Edge::Edge(VertexId from, VertexId to) : from(from), to(to) {
}

template <class Iterator>
IteratorRange<Iterator>::IteratorRange(Iterator begin, Iterator end)
    : begin_(std::move(begin))
    , end_(std::move(end)) {
}

template <class Iterator>
Iterator IteratorRange<Iterator>::begin() const {  // NOLINT
    return begin_;
}

template <class Iterator>
Iterator IteratorRange<Iterator>::end() const {  // NOLINT
    return end_;
}

template <class Predicate, class Iterator>
FilterIterator<Predicate, Iterator>::FilterIterator(
    Iterator current,
    Iterator end,
    Predicate predicate)
    : current_(std::move(current))
    , end_(std::move(end))
    , predicate_(std::move(predicate)) {
    FindNextIterator();
}

template <class Predicate, class Iterator>
FilterIterator<Predicate, Iterator>& FilterIterator<Predicate, Iterator>::operator++() {
    ++current_;
    FindNextIterator();
    return *this;
}

template <class Predicate, class Iterator>
const auto& FilterIterator<Predicate, Iterator>::operator*() const {
    return *current_;
}

template <class Predicate, class Iterator>
bool FilterIterator<Predicate, Iterator>::operator==(const FilterIterator& rhs) const {
    return current_ == rhs.current_;
}

template <class Predicate, class Iterator>
bool FilterIterator<Predicate, Iterator>::operator!=(const FilterIterator& rhs) const {
    return !(*this == rhs);
}

template <class Predicate, class Iterator>
void FilterIterator<Predicate, Iterator>::FindNextIterator() {
    while (current_ != end_ && !predicate_(*current_)) {
        ++current_;
    }
}

void Graph::AddEdge(VertexId from, VertexId to) {
    auto edge_id = edges_.size();
    adjacency_lists_[from].push_back(edge_id);
    edges_.emplace_back(from, to);
}

void Graph::AddVertex() {
    adjacency_lists_.emplace_back();
}

auto Graph::GetOutEdgeIds(VertexId vertex) const {
    return IteratorRange(
        adjacency_lists_[vertex].cbegin(),
        adjacency_lists_[vertex].cend());
}

VertexId Graph::Source(EdgeId edge_id) const {
    return edges_[edge_id].from;
}

VertexId Graph::Target(EdgeId edge_id) const {
    return edges_[edge_id].to;
}

VertexId Graph::VerticesNumber() const {
    return adjacency_lists_.size();
}

template <class Predicate>
FilteredGraph<Predicate>::FilteredGraph(const Graph& graph, Predicate predicate)
    : graph_(graph)
    , predicate_(std::move(predicate)) {
}

template <class Predicate>
auto FilteredGraph<Predicate>::GetOutEdgeIds(VertexId vertex) const {
    auto edges_range = graph_.GetOutEdgeIds(vertex);
    return IteratorRange(
        FilterIterator(edges_range.begin(), edges_range.end(), predicate_),
        FilterIterator(edges_range.end(), edges_range.end(), predicate_));
}

template <class Predicate>
VertexId FilteredGraph<Predicate>::Source(EdgeId edge_id) const {
    return graph_.Source(edge_id);
}

template <class Predicate>
VertexId FilteredGraph<Predicate>::Target(EdgeId edge_id) const {
    return graph_.Target(edge_id);
}

template <class Predicate>
VertexId FilteredGraph<Predicate>::VerticesNumber() const {
    return graph_.VerticesNumber();
}

template <class GraphType>
ShortestPathBfsVisitor<GraphType>::ShortestPathBfsVisitor(
    const GraphType& graph,
    VertexId source,
    VertexId target,
    std::vector<EdgeId>* path)
    : graph_(graph)
    , source_(source)
    , target_(target)
    , parent_(graph_.VerticesNumber())
    , path_(path) {
}

template <class GraphType>
void ShortestPathBfsVisitor<GraphType>::DiscoverVertex(VertexId vertex) {
    if (vertex == target_) {
        while (vertex != source_) {
            auto parent_edge_id = parent_[vertex];
            path_->push_back(parent_edge_id);
            vertex = graph_.Source(parent_edge_id);
        }
        std::reverse(path_->begin(), path_->end());
    }
}

template <class GraphType>
void ShortestPathBfsVisitor<GraphType>::TreeEdge(EdgeId edge_id) {
    parent_[graph_.Target(edge_id)] = edge_id;
}

void FlowNetwork::AddFlow(EdgeId edge_id, int flow) {
    edge_properties_[edge_id].flow += flow;
    edge_properties_[BackwardEdgeId(edge_id)].flow -= flow;
}

int FlowNetwork::GetFlow() const {
    int flow = 0;
    for (auto edge_id : graph_.GetOutEdgeIds(source_)) {
        flow += edge_properties_[edge_id].flow;
    }
    return flow;
}

int FlowNetwork::GetMaxPossibleFlow() const {
    int flow = 0;
    for (auto edge_id : graph_.GetOutEdgeIds(source_)) {
        flow += edge_properties_[edge_id].capacity;
    }
    return flow;
}

VertexId FlowNetwork::GetSource() const {
    return source_;
}

VertexId FlowNetwork::GetSink() const {
    return sink_;
}

int FlowNetwork::ResidualCapacity(EdgeId edge_id) const {
    return edge_properties_[edge_id].capacity - edge_properties_[edge_id].flow;
}

auto FlowNetwork::ResidualNetworkView() const {
    auto predicate = [this](EdgeId edge_id) {
        return ResidualCapacity(edge_id) > 0;
    };

    return FilteredGraph(graph_, predicate);
}

EdgeId FlowNetwork::BackwardEdgeId(EdgeId edge_id) const {
    return edge_id ^ static_cast<EdgeId>(1);
}

FlowNetwork::EdgeProperties::EdgeProperties(int capacity) : capacity(capacity) {
}

void FlowNetworkBuilder::AddEdgeWithCapacity(const Edge& edge, int capacity) {
    network_.graph_.AddEdge(edge.from, edge.to);
    network_.edge_properties_.emplace_back(capacity);

    network_.graph_.AddEdge(edge.to, edge.from);
    network_.edge_properties_.emplace_back(/* capacity = */ 0);
}

FlowNetwork&& FlowNetworkBuilder::Build() {
    return std::move(network_);
}

void FlowNetworkBuilder::SetSink(VertexId sink) {
    network_.sink_ = sink;
}

void FlowNetworkBuilder::SetSource(VertexId source) {
    network_.source_ = source;
}

void FlowNetworkBuilder::SetVerticesNumber(VertexId vertices_number) {
    for (VertexId vertex = 0; vertex < vertices_number; ++vertex) {
        network_.graph_.AddVertex();
    }
}

EdmondsKarp::EdmondsKarp(FlowNetwork* network) : network_(network) {
}

int EdmondsKarp::FindMaxFlow() {
    auto source = network_->GetSource();
    auto sink = network_->GetSink();
    std::vector<EdgeId> path;

    const auto& residual_network = network_->ResidualNetworkView();
    TraverseGraphInBfsOrder(
        residual_network,
        source,
        ShortestPathBfsVisitor(residual_network, source, sink, &path));
    while (!path.empty()) {
        UpdateFlow(path);
        path.clear();
        TraverseGraphInBfsOrder(
            residual_network,
            source,
            ShortestPathBfsVisitor(residual_network, source, sink, &path));
    }
    return network_->GetFlow();
}

void EdmondsKarp::UpdateFlow(const std::vector<EdgeId>& path) {
    auto min_capacity_edge_id = *std::min_element(
        path.begin(),
        path.end(),
        [this](EdgeId lhs, EdgeId rhs) {
            return network_->ResidualCapacity(lhs) < network_->ResidualCapacity(rhs);
        });

    auto min_capacity = network_->ResidualCapacity(min_capacity_edge_id);
    for (auto edge_id : path) {
        network_->AddFlow(edge_id, min_capacity);
    }
}

std::istream& operator>>(std::istream& is, Edge& edge) {
    is >> edge.from >> edge.to;
    return is;
}

int FindMinMaxGoldBars(
    int people_number,
    int vertices_number,
    int source,
    int sink,
    const std::vector<int>& gold_bars,
    const std::vector<Edge>& trust_relations) {
    FlowNetworkBuilder initial_builder;
    initial_builder.SetVerticesNumber(vertices_number);
    initial_builder.SetSource(source);
    initial_builder.SetSink(sink);
    for (int human = 1; human <= people_number; ++human) {
        initial_builder.AddEdgeWithCapacity(Edge(source, human), gold_bars[human - 1]);
    }
    for (const auto& edge : trust_relations) {
        initial_builder.AddEdgeWithCapacity(edge, std::numeric_limits<int>::max());
    }

    auto has_max_flow = [&initial_builder, people_number, sink](int sink_edge_capacity) {
        auto network_builder = initial_builder;
        for (int human = 1; human <= people_number; ++human) {
            network_builder.AddEdgeWithCapacity(Edge(human, sink), sink_edge_capacity);
        }
        auto network = network_builder.Build();

        auto flow = EdmondsKarp(&network).FindMaxFlow();

        return network.GetMaxPossibleFlow() == flow;
    };

    return BinarySearch(kMinPossibleAnswer, kMaxPossibleAnswer, has_max_flow);
}
