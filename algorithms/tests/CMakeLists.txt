include_directories(../)
include_directories(../../graph)

link_libraries(algorithms)

add_executable(
    test_multilevel_graph
    multilevel_graphs.cpp
    test_multilevel_graph.cpp
)

add_executable(
    test_multilevel_dijkstra
    multilevel_graphs.cpp
    test_multilevel_dijkstra.cpp
)

add_executable(
    test_topology_builders
    multilevel_graphs.cpp
    test_topology_builders.cpp
)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost 1.71.0 REQUIRED COMPONENTS graph)

include_directories(${Boost_INCLUDE_DIRS})
add_executable(
    stress_tests
    stress_test_dijkstra.cpp
)

target_link_libraries(stress_tests ${Boost_LIBRARIES})
