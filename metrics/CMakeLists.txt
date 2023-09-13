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
