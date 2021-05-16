add_executable(
    test_dump_and_load
    test_dump_and_load.cpp
)
add_dependencies(test_dump_and_load make_directory)

target_include_directories(test_dump_and_load PUBLIC ../)
target_link_libraries(test_dump_and_load graph)
