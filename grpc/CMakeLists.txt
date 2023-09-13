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

# Set Proto file
include(gen_proto.cmake)

# generate_proto("../protos/helloworld.proto")
generate_proto("../protos/tools_interface.proto")

include_directories(include)

add_library(${COMPONENT_NAME}-lib metric_services.cpp)
target_link_libraries(${COMPONENT_NAME}-lib tools_interface_proto_lib metrics)

# Point cloud processing server
add_executable(${COMPONENT_NAME}-server tools_grpc_server.cpp)
target_link_libraries(${COMPONENT_NAME}-server ${COMPONENT_NAME}-lib )

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