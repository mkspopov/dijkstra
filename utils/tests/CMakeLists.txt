include_directories(../)
add_executable(
    test_utils
    test_utils.cpp
)
target_link_libraries(test_utils utils)
