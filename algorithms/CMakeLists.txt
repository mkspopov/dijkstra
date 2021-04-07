add_library(
    algorithms
    STATIC
    bidirectional_dijkstra.h bidirectional_dijkstra.cpp
    dijkstra.h dijkstra.cpp
    dijkstra_prevrun.h
    heap.h
    shortest_path_algorithm.h shortest_path_algorithm.cpp
    visitor.h visitor.cpp
)

target_link_libraries(algorithms graph)
