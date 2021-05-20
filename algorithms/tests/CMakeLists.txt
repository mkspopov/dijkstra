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
endfunction()

AddTest(test_multilevel_dijkstra)
AddTest(test_topology_builders)
AddTest(test_mlg_dump_and_load)
AddTest(test_inertial_flow)
AddTest(stress_test_dijkstra)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.71.0 REQUIRED COMPONENTS graph)
target_link_libraries(stress_test_dijkstra ${Boost_LIBRARIES})
