cmake_minimum_required(VERSION 3.17)
project(dijkstra)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined -Wall -Werror -Wextra")

add_executable(
    dijkstra

    main.cpp
    graph/graph.cpp
    graph/graph.h
    graph/serializer.cpp
    graph/serializer.h
    utils.cpp
    utils.h
    algorithms/dijkstra.cpp
    algorithms/visitor.cpp)
