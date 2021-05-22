add_library(
    utils
    STATIC
    utils.cpp utils.h
    thread_pool.cpp thread_pool.h
    types.h
)

add_subdirectory(tests)
