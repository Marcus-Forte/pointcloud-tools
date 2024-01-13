set (COMPONENT_NAME "grpc-interface")

find_package(absl PATHS ${GRPC_INSTALL_PATH} CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")
find_package(gRPC PATHS ${GRPC_INSTALL_PATH} CONFIG REQUIRED)
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_REFLECTION gRPC::grpc++_reflection)
find_program(_PROTOBUF_PROTOC protoc)
if(NOT _PROTOBUF_PROTOC)
    message (FATAL_ERROR "protoc not found")
endif()

find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
if(NOT _GRPC_CPP_PLUGIN_EXECUTABLE)
    message (FATAL_ERROR "grpc_cpp_plugin not found")
endif()

# Proto gereration function
include(gen_proto.cmake)

generate_proto("${CMAKE_CURRENT_SOURCE_DIR}/protos/common.proto")
generate_proto("${CMAKE_CURRENT_SOURCE_DIR}/protos/metrics.proto")
generate_proto("${CMAKE_CURRENT_SOURCE_DIR}/protos/filters.proto")
generate_proto("${CMAKE_CURRENT_SOURCE_DIR}/protos/reconstruct.proto")
generate_proto("${CMAKE_CURRENT_SOURCE_DIR}/protos/tools.proto")


include_directories(include)

add_library(${COMPONENT_NAME}-proto-lib
${CMAKE_CURRENT_BINARY_DIR}/common.pb.cc
${CMAKE_CURRENT_BINARY_DIR}/common.grpc.pb.cc

${CMAKE_CURRENT_BINARY_DIR}/metrics.pb.cc
${CMAKE_CURRENT_BINARY_DIR}/metrics.grpc.pb.cc

${CMAKE_CURRENT_BINARY_DIR}/filters.pb.cc
${CMAKE_CURRENT_BINARY_DIR}/filters.grpc.pb.cc

${CMAKE_CURRENT_BINARY_DIR}/reconstruct.pb.cc
${CMAKE_CURRENT_BINARY_DIR}/reconstruct.grpc.pb.cc

${CMAKE_CURRENT_BINARY_DIR}/tools.pb.cc
${CMAKE_CURRENT_BINARY_DIR}/tools.grpc.pb.cc
)

target_link_libraries(${COMPONENT_NAME}-proto-lib
${_REFLECTION}
${_GRPC_GRPCPP}
${_PROTOBUF_LIBPROTOBUF}
)

set(grpc_lib_src
src/metric_services.cpp
src/filter_services.cpp
src/las_conversions.cpp
)

set(colmap_lib)
if(WITH_COLMAP)
    find_package(colmap 3.9.1 REQUIRED)
    set(grpc_lib_src ${grpc_lib_src} src/reconstruct_services.cpp)
    set(colmap_lib colmap::colmap)
    add_compile_definitions(WITH_COLMAP)
endif()
message(STATUS "gRPC component sources: ${grpc-lib-src}")

add_library(${COMPONENT_NAME}-lib 
${grpc_lib_src}
)

target_link_libraries(${COMPONENT_NAME}-lib 
${COMPONENT_NAME}-proto-lib
metrics 
las
${colmap_lib})

add_executable(${COMPONENT_NAME}-server src/tools_grpc_server.cpp)
target_link_libraries(${COMPONENT_NAME}-server ${COMPONENT_NAME}-lib )

if(BUILD_TESTS)
    add_definitions(-DTEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tst/tst_data")
    find_package(GTest REQUIRED)
    enable_testing()
    include(GoogleTest)
    add_executable(test_service 
    tst/test_main.cpp
    tst/test_service.cpp)
    target_link_libraries(test_service ${COMPONENT_NAME}-lib GTest::gtest)
    gtest_discover_tests(test_service)
endif()

# Component installation.
install(
TARGETS ${COMPONENT_NAME}-server
EXPORT ${COMPONENT_NAME}-config
)

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    EXPORT ${COMPONENT_NAME}-config
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)