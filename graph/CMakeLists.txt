add_library(
    graph
    STATIC
    graph.h graph.cpp
    multilevel_graph.h multilevel_graph.cpp
    serializer.h serializer.cpp
    topology.h topology.cpp
)

target_include_directories(graph PUBLIC ../utils)
target_link_libraries(graph messages utils)

add_subdirectory(tests)
