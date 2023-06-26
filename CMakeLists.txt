cmake_minimum_required(VERSION 3.10)
project(duna-pointcloud-tools)
set(CMAKE_CXX_STANDARD 17)
find_package(PCL 1.13 REQUIRED COMPONENTS common io filters search visualization)
find_package(duna-optimizer REQUIRED)
find_package(OpenMP REQUIRED)
find_package(tsl-robin-map REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

if(UNIX)
# GTest required pthread in UNIX Os
        link_libraries(pthread)
        if(LINUX) # not APPLE
        add_compile_options(-msse -msse2 -mavx -mfma -msse4)
        add_compile_options(-march=native)
        endif()
elseif(MSVC)
        add_compile_options(/bigobj)
        add_compile_definitions(_USE_MATH_DEFINES)
endif()

link_libraries(${PCL_LIBRARIES})

# Protobuf & gRPC definitions
list (APPEND CMAKE_PREFIX_PATH "/home/marcus/workspace/cpp/grpc/build/install/") # <-- Set this accordingingly!
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Declare options
option(BUILD_FILTERS "Build filter functions" ON)
option(BUILD_GRPC "Build GRPC API functions" OFF)
option(BUILD_MAPPING "Build mapping " ON)
option(BUILD_SCANMATCHING "Build scan matching " ON)
option(BUILD_TESTS "Build tests" OFF)
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
if(BUILD_MAPPING)
add_subdirectory(mapping)
endif()
if(BUILD_SCANMATCHING)
add_subdirectory(scan_matching)
endif()

if(BUILD_TESTS)

endif()

# Filters
configure_file(${CMAKE_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in ${PROJECT_NAME}-config.cmake
               @ONLY)

install(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})