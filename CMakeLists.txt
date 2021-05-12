cmake_minimum_required(VERSION 3.17)
project(dijkstra)

set(CMAKE_CXX_STANDARD 20)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined -Wall -Werror -Wextra")

add_executable(
    dijkstra
    main.cpp
)

add_subdirectory(algorithms)
target_link_libraries(dijkstra algorithms)

add_subdirectory(graph)
target_link_libraries(dijkstra graph)

add_subdirectory(perf)
add_subdirectory(utils)

add_executable(my my.cpp)
