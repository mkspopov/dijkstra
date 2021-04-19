set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost 1.71.0 REQUIRED COMPONENTS graph)

include_directories(${Boost_INCLUDE_DIRS})
add_executable(
    perf_tests
    stress_test_dijkstra.cpp
)

target_link_libraries(perf_tests ${Boost_LIBRARIES})
target_link_libraries(perf_tests algorithms)

include(FetchContent)
FetchContent_Declare(
    daw_json_link
    GIT_REPOSITORY https://github.com/beached/daw_json_link
    GIT_TAG release
)
FetchContent_MakeAvailable(daw_json_link)

target_link_libraries(perf_tests daw::json_link)
