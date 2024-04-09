cmake_minimum_required(VERSION 3.10)
project(duna-pointcloud-tools)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PCL 1.13 REQUIRED COMPONENTS common io filters search surface features visualization)
find_package(moptimizer REQUIRED)
find_package(tsl-robin-map REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

if(UNIX)
        if(NOT APPLE)
        add_compile_options(-march=native)
        endif()
elseif(MSVC)
        add_compile_options(/bigobj)
        add_compile_definitions(_USE_MATH_DEFINES)
endif()

# Declare options
option(BUILD_FILTERS "Build filter functions" ON)
option(BUILD_GRPC "Build GRPC API functions" OFF)
option(BUILD_MAPPING "Build mapping " ON)
option(BUILD_SCANMATCHING "Build scan matching " ON)
option(BUILD_TESTS "Build tests" OFF)
option(WITH_COLMAP "With colmap" OFF)

if(BUILD_TESTS)
enable_testing()
endif()

include(GNUInstallDirs)
if (BUILD_FILTERS)
    add_subdirectory(filters)
endif()
if(BUILD_GRPC)
    add_subdirectory(grpc)
    # libLAS config
    option(WITH_GEOTIFF "" OFF)
    option(WITH_GDAL "" OFF)
    option(WITH_TESTS "" OFF)
    add_subdirectory(third_party/libLAS)
endif()
if(BUILD_MAPPING)
    add_subdirectory(mapping)
endif()
if(BUILD_SCANMATCHING)
    add_subdirectory(scan_matching)
endif()

add_subdirectory(metrics)

# Filters
configure_file(${CMAKE_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in ${PROJECT_NAME}-config.cmake
               @ONLY)

install(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})