cmake_minimum_required(VERSION 3.10)
project(duna-cloud-tools)

find_package(PCL 1.10 REQUIRED COMPONENTS common io filters)
find_package(OpenMP REQUIRED)

link_libraries(${PCL_LIBRARIES})

# Protobuf & gRPC definitions
list (APPEND CMAKE_PREFIX_PATH "/home/marcus/workspace/cpp/grpc/build/install/") # <-- Set this accordingingly!
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Declare options
option(BUILD_FILTERS "Build filter functions" ON)
option(BUILD_GRPC "Build GRPC API functions" OFF)

# OLD
# include_directories(include)
# add_library(global_registrator
# lib/global_registration)

# add_library(asl_parser 
# lib/asl_parser.cpp)


# add_executable(keypoint src/keypoint.cpp)
# add_executable(keypoint_registration src/keypoint_registration.cpp)

# target_link_libraries(keypoint asl_parser global_registrator)
# target_link_libraries(keypoint_registration asl_parser global_registrator )


# # Batch processors
# add_executable(batch_voxel src/batch_voxel.cpp)
# add_executable(batch_sor src/batch_sor.cpp )
# add_executable(batch_ror src/batch_ror.cpp )

include(GNUInstallDirs)
if (BUILD_FILTERS)
add_subdirectory(filters)
endif()
if(BUILD_GRPC)
add_subdirectory(grpc)
endif()

# Filters
configure_file(${CMAKE_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in ${PROJECT_NAME}-config.cmake
               @ONLY)

install(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})