function (generate_proto proto_file)
# Generates a protobuf library for a given .proto file. 
# The library shall be named after the name of the file and a "_proto_lib" suffix.

if (NOT EXISTS ${proto_file})
message(FATAL_ERROR "generate_proto: '.proto' file '${proto_file}' not found.")
endif()

get_filename_component(protofile_fullpath "${proto_file}" ABSOLUTE)
get_filename_component(protofile_path "${protofile_fullpath}" PATH)
get_filename_component(protofile_name "${protofile_fullpath}" NAME_WE)

message(STATUS "Generating protolib for ${protofile_fullpath}")
# message(STATUS "Generating protolib for ${protofile_path}")
# message(STATUS "Generating protolib for ${protofile_name}")

# Generated sources
set(proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/${protofile_name}.pb.cc")
set(proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/${protofile_name}.pb.h")
set(proto_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/${protofile_name}.grpc.pb.cc")
set(proto_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/${protofile_name}.grpc.pb.h")

add_custom_command(
      OUTPUT "${proto_srcs}" "${proto_hdrs}" "${proto_grpc_srcs}" "${proto_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${protofile_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${protofile_fullpath}"
      DEPENDS "${protofile_fullpath}"
      )

add_library("${protofile_name}_proto_lib" SHARED
      ${proto_srcs}
      ${proto_hdrs}
      ${proto_grpc_srcs}
      ${proto_grpc_hdrs})

target_link_libraries("${protofile_name}_proto_lib"
      ${_REFLECTION}
      ${_GRPC_GRPCPP}
      ${_PROTOBUF_LIBPROTOBUF})

# Have the generated header files be found.
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

endfunction()