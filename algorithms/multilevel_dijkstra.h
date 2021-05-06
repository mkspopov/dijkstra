//#pragma once
//
//#include "dijkstra.h"
//#include "multilevel_graph.h"
//
///*
// * MultiLevelGraph is not used to calculate single-to-all shortest paths.
// * That is, source and target vertices are known from the beginning.
// */
//
//LevelId Level(
//    const MultilevelGraph& graph,
//    VertexId source,
//    VertexId target,
//    VertexId vertex)
//{
//    return std::min(graph.MaxDistinctLevel(source, vertex), graph.MaxDistinctLevel(target, vertex));
//}
//
//template <class Queue, class Visitor>
//void MultiLevelDijkstra(
//    const MultilevelGraph& graph,
//    std::vector<Weight>& distances,
//    std::vector<Color>& colors,
//    const std::vector<VertexId>& sources,
//    const std::vector<VertexId>& targets,
//    Queue& queue,
//    Visitor& visitor)
//{
//    for (auto source : sources) {
//        distances[source] = 0;
//        colors[source] = Color::GRAY;
//        queue.Push(source, 0);
//        visitor.SourceVertex(source);
//    }
//
//    while (!queue.Empty()) {
//        auto [from, dist] = queue.Extract();
//        // TODO: Optional check to easily deal with
//        // std::priority_queue. This may slow down
//        // code because of memory load of a random vertex
//        // of the graph. So, measure time without it.
//        if (colors[from] == Color::BLACK) {
//            continue;
//        }
//        visitor.ExamineVertex(from);
//
//        auto level = Level(graph, sources[0], targets[0], from);
//        auto cell = graph.Cell(from, level);
//
//        for (auto edgeId : graph.GetOutgoingEdges(cell)) {
//            visitor.ExamineEdge(edgeId);
//            auto to = graph.GetTarget(edgeId);
//            auto relaxedDist = dist + graph.GetEdgeProperties(edgeId).weight;
//            if (relaxedDist < distances[to]) {
//                distances[to] = relaxedDist;
//                colors[to] = Color::GRAY;
//                if (colors[to] == Color::WHITE) {
//                    queue.Push(to, relaxedDist);
//                } else if (colors[to] == Color::GRAY) {
//                    queue.Decrease(to, relaxedDist);
//                }
//                visitor.EdgeRelaxed(edgeId);
//            }
//        }
//        colors[from] = Color::BLACK;
//        visitor.FinishVertex(from);
//    }
//}
//
//template <class Queue, class Visitor>
//auto MultiLevelDijkstra(
//    const MultilevelGraph& graph,
//    const std::vector<VertexId>& sources,
//    const std::vector<VertexId>& targets,
//    Visitor& visitor)
//{
//    std::vector<Weight> distances(graph.VerticesCount(), Dijkstra::INF);
//    std::vector<Color> colors(graph.VerticesCount(), Color::WHITE);
//    Queue queue;
//    BoostDijkstra(graph, distances, colors, sources, targets, queue, visitor);
//    return distances;
//}
