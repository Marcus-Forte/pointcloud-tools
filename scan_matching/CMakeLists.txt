set (COMPONENT_NAME "scan_matching")

message(STATUS "Processing component: ${CMAKE_CURRENT_LIST_DIR}")

add_library(${COMPONENT_NAME} INTERFACE)

target_include_directories(${COMPONENT_NAME} INTERFACE 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

target_link_libraries(${COMPONENT_NAME} INTERFACE mapping duna::duna-optimizer)

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
    get_filename_component(DATASET_PATH tst/data/ REALPATH)
    add_definitions(-DTEST_DATA_DIR="${DATASET_PATH}")
    set(tst_srcs
    # tst/registration_point2plane.cpp
    tst/registration_point2point3DOF.cpp
    tst/registration_point2point.cpp
    tst/sequence_3dof.cpp
    )
    foreach(test_case ${tst_srcs})
        get_filename_component(test_src_name ${test_case} NAME_WE)
        message(STATUS ${filename})    
        add_executable(${test_src_name} ${test_case})
        target_link_libraries(${test_src_name} ${GTEST_LIBRARIES} ${COMPONENT_NAME} )
        gtest_discover_tests(${test_src_name})
    endforeach()
endif()