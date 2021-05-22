add_executable(
    perf
    perf.cpp
)

add_dependencies(perf make_directory)

target_include_directories(perf PUBLIC ../algorithms)
target_link_libraries(perf algorithms)
target_link_libraries(perf utils_for_tests)
