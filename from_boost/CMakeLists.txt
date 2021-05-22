set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost 1.71.0 REQUIRED COMPONENTS graph)

include_directories(${Boost_INCLUDE_DIRS})
add_library(
    utils_for_tests
    utils_for_tests.h utils_for_tests.cpp
)

target_include_directories(utils_for_tests PUBLIC ../graph)
target_link_libraries(utils_for_tests ${Boost_LIBRARIES})
target_link_libraries(utils_for_tests graph)
