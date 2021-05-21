cmake_minimum_required(VERSION 3.18)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_COMPILER "g++-10")

project(dijkstra)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -pthread -std=c++20")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined -Wall -Werror -Wextra -pthread -std=c++20")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread -Wall -Werror -Wextra -pthread -std=c++20")

enable_testing()

add_custom_target(make_directory ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory /tmp/tests/
)

add_executable(
    dijkstra
    main.cpp
)

add_subdirectory(algorithms)
target_link_libraries(dijkstra algorithms)

add_subdirectory(graph)
target_link_libraries(dijkstra graph)

add_subdirectory(perf)
add_subdirectory(from_boost)
add_subdirectory(utils)

add_executable(my my.cpp)

# Build Types
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE}
    FORCE
)
set(CMAKE_CXX_FLAGS_ASAN
    "-fsanitize=address,undefined -fno-optimize-sibling-calls -fsanitize-address-use-after-scope -fno-omit-frame-pointer -g -O1"
    FORCE
)
set(CMAKE_CXX_FLAGS_TSAN
    "-fsanitize=thread -g -O1"
    FORCE
)
