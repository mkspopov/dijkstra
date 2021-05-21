set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost 1.71.0 REQUIRED COMPONENTS graph)

include_directories(${Boost_INCLUDE_DIRS})
add_library(
    test_utils
    test_utils.h test_utils.cpp
)

target_include_directories(test_utils PUBLIC ../graph)
target_link_libraries(test_utils ${Boost_LIBRARIES})
target_link_libraries(test_utils graph)
#target_link_libraries(perf algorithms)
