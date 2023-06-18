set (COMPONENT_NAME "mapping")

message(STATUS "Processing component: ${CMAKE_CURRENT_LIST_DIR}")

set(SRCS 
src/voxel_hashing_map.cpp)

add_library(${COMPONENT_NAME} SHARED ${SRCS})

target_include_directories(${COMPONENT_NAME} PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

# Component installation.
install(
TARGETS ${COMPONENT_NAME} 
EXPORT ${COMPONENT_NAME}-config
)

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    EXPORT ${COMPONENT_NAME}-config
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()
    include(GoogleTest)
    message(STATUS "Building tests for component '${COMPONENT_NAME}' ")
    set(tst_srcs
    tst/tst_insert.cpp)
    foreach(test_case ${tst_srcs})
        get_filename_component(test_src_name ${test_case} NAME_WE)
        message(STATUS ${filename})    
        add_executable(${test_src_name} ${test_case})
        target_link_libraries(${test_src_name} ${COMPONENT_NAME} ${GTEST_LIBRARIES})
        gtest_discover_tests(${test_src_name})
    endforeach()
endif()