cmake_minimum_required(VERSION 3.17)
project(dijkstra)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")

add_executable(
    dijkstra

    main.cpp
    graph/graph.cpp
    graph/graph.h
    graph/serializer.cpp
    graph/serializer.h
    utils.cpp
    utils.h
        algorithms/dijkstra.cpp algorithms/dijkstra.h algorithms/heap.h)
