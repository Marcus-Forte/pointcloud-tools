set (COMPONENT_NAME "filters")

message(STATUS "Processing component: ${CMAKE_CURRENT_LIST_DIR}")

set(SRCS 
src/voxel_max.cpp)

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