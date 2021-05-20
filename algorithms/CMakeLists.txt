add_library(
    algorithms
    STATIC
    bidirectional_dijkstra.h bidirectional_dijkstra.cpp
    contraction.h contraction.cpp
    dijkstra.h dijkstra.cpp
    flow_network.h flow_network.cpp
    heap.h
    inertial_flow.h inertial_flow.cpp
    min_cut.h min_cut.cpp
    multilevel_dijkstra.h multilevel_dijkstra.cpp
    shortest_path_algorithm.h shortest_path_algorithm.cpp
    topology_builders.h topology_builders.cpp
    visitor.h visitor.cpp
)

target_include_directories(algorithms PUBLIC ../graph)
target_link_libraries(algorithms graph)

add_subdirectory(tests)
