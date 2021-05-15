cmake_minimum_required(VERSION 3.17)
project(dijkstra)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -pthread -std=c++20")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined -Wall -Werror -Wextra -pthread")

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

add_custom_target(build-time-make-directory ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory /tmp/tests/
)
