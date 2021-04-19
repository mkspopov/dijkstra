add_library(
    graph
    STATIC
    graph.h graph.cpp
    serializer.h serializer.cpp
)

target_link_libraries(graph utils)
