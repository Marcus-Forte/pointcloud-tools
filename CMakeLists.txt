cmake_minimum_required(VERSION 3.20)
project(pointcloud-tools)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.13 REQUIRED COMPONENTS common io filters search surface features visualization)
find_package(moptimizer REQUIRED)
find_package(tsl-robin-map REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)

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
    option(WITH_UTILITIES "" OFF)
    add_subdirectory(third_party/libLAS)
endif()
if(BUILD_MAPPING)
    add_subdirectory(mapping)
endif()
if(BUILD_SCANMATCHING)
    add_subdirectory(scan_matching)
endif()

add_subdirectory(metrics)

find_program(FORMATTER clang-format NO_CACHE)

if(NOT FORMATTER)
message(STATUS "clang-format NOT detected.")
else()
message(STATUS "clang-format detected. Applying code format...")
add_custom_target(formatting ALL
COMMAND bash -c "find \
${CMAKE_SOURCE_DIR}/src \
${CMAKE_SOURCE_DIR}/scan_matching \
${CMAKE_SOURCE_DIR}/registration \
${CMAKE_SOURCE_DIR}/metrics \
${CMAKE_SOURCE_DIR}/mapping \
${CMAKE_SOURCE_DIR}/grpc \
${CMAKE_SOURCE_DIR}/filters \
 -name *.cpp -o -name *.h | xargs clang-format -i"
 VERBATIM)
endif()

# Filters
configure_file(${CMAKE_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in ${PROJECT_NAME}-config.cmake
               @ONLY)

install(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})