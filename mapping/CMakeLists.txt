set (COMPONENT_NAME "mapping")

message(STATUS "Processing component: ${CMAKE_CURRENT_LIST_DIR}")

set(SRCS 
src/VoxelHashMap.cpp
)

add_library(${COMPONENT_NAME} SHARED ${SRCS})

target_include_directories(${COMPONENT_NAME} PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

target_link_libraries(${COMPONENT_NAME} tsl::robin_map ${PCL_LIBRARIES} TBB::tbb)

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
    include(GoogleTest)
    message(STATUS "Building tests for component '${COMPONENT_NAME}' ")

    add_executable(tst_mapping 
    tst/test_voxel_mapping.cpp
    tst/test_kdtree_mapping.cpp)
    target_link_libraries(tst_mapping PRIVATE ${COMPONENT_NAME} GTest::gtest_main)
    gtest_discover_tests(tst_mapping)
endif()