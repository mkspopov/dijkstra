include_directories(../)
include_directories(../../graph)

link_libraries(algorithms)

function(AddTest name)
    add_executable(
        ${name}
        multilevel_graphs.cpp
        ${name}.cpp
    )
    add_dependencies(${name} make_directory)
    add_test(${name} ${name})
endfunction()

AddTest(test_topology_builders)
AddTest(test_multilevel_graph)
AddTest(test_mlg_dump_and_load)
AddTest(test_multilevel_dijkstra)
AddTest(test_inertial_flow)
AddTest(stress_test_dijkstra)

target_link_libraries(stress_test_dijkstra utils_for_tests)

set_tests_properties(
    stress_test_dijkstra
    PROPERTIES TIMEOUT 300
)

set_tests_properties(
    test_inertial_flow
    test_mlg_dump_and_load
    test_multilevel_graph
    test_multilevel_dijkstra
    test_topology_builders
    PROPERTIES TIMEOUT 1
)
