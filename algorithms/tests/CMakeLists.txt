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
add_dependencies(test_multilevel_dijkstra make_directory)

add_executable(
    test_topology_builders
    multilevel_graphs.cpp
    test_topology_builders.cpp
)

add_executable(
    test_mlg_dump_and_load
    multilevel_graphs.cpp
    test_mlg_dump_and_load.cpp
)
add_dependencies(test_mlg_dump_and_load make_directory)

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
add_dependencies(stress_tests make_directory)
