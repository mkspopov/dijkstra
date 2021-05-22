add_library(
    graph
    STATIC
    geometry.h geometry.cpp
    graph.h graph.cpp
    serializer.h serializer.cpp
    topology.h topology.cpp
)

target_include_directories(graph PUBLIC ../utils)
target_link_libraries(graph utils)

add_subdirectory(tests)
