set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost 1.71.0 REQUIRED COMPONENTS graph)

include_directories(${Boost_INCLUDE_DIRS})
add_executable(
    tests
    test_dijkstra.cpp
)

target_link_libraries(tests ${Boost_LIBRARIES})
target_link_libraries(tests algorithms)
