set (COMPONENT_NAME "metrics")

message(STATUS "Processing component: ${CMAKE_CURRENT_LIST_DIR}")

set(SRCS 
src/metrics.cpp
)

add_library(${COMPONENT_NAME} SHARED ${SRCS})
target_include_directories(${COMPONENT_NAME} PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
target_link_libraries(${COMPONENT_NAME} ${PCL_LIBRARIES})


if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()
    include(GoogleTest)
    add_executable(test_metrics
    tst/test_compute_area.cpp)
    target_link_libraries(test_metrics ${COMPONENT_NAME} GTest::gtest)
    gtest_discover_tests(test_metrics)

    # copy test file to build
    configure_file(tst/test_surface.ply ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
endif()
