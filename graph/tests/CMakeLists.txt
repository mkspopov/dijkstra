add_executable(
    test_dump_and_load
    test_dump_and_load.cpp
)

target_include_directories(test_dump_and_load PUBLIC ../)
target_link_libraries(test_dump_and_load graph)
