cmake_minimum_required(VERSION 3.18)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_COMPILER "g++-10")

project(dijkstra)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -pthread -std=c++20")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined -Wall -Werror -Wextra -pthread")

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

#add_subdirectory(perf)
add_subdirectory(utils)

add_executable(my my.cpp)

enable_testing()

add_test(test_dump_and_load graph/tests/test_dump_and_load)
add_test(test_mlg_dump_and_load algorithms/tests/test_mlg_dump_and_load)
add_test(test_topology_builders algorithms/tests/test_topology_builders)
add_test(test_multilevel_graph algorithms/tests/test_multilevel_graph)
add_test(test_multilevel_dijkstra algorithms/tests/test_multilevel_dijkstra)

set_tests_properties(
    test_dump_and_load
    test_mlg_dump_and_load
    test_topology_builders
    test_multilevel_graph
    test_multilevel_dijkstra
    PROPERTIES TIMEOUT 1
)

add_test(stress_test algorithms/tests/stress_tests)
set_tests_properties(
    stress_test
    PROPERTIES TIMEOUT 300
)

# Build Types
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE}
    CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel tsan asan lsan msan ubsan"
    FORCE
)

set(CMAKE_CXX_FLAGS_ASAN
    "-fsanitize=address,undefined -fno-optimize-sibling-calls -fsanitize-address-use-after-scope -fno-omit-frame-pointer -g -O1"
    CACHE STRING "Flags used by the C++ compiler during AddressSanitizer builds."
    FORCE
)

set(CMAKE_CXX_FLAGS_TSAN
    "-fsanitize=thread -g -O1"
    CACHE STRING "Flags used by the C++ compiler during ThreadSanitizer builds."
    FORCE
)
