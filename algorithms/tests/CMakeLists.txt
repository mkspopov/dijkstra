add_executable(
    graph_tests
    test_multilevel_graph.cpp
)

target_link_libraries(graph_tests graph)
target_include_directories(graph_tests PUBLIC ../)
target_include_directories(graph_tests PUBLIC ../../graph)
