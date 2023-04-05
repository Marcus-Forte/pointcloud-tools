cmake_minimum_required(VERSION 3.22)
project(test)

find_package(duna-cloud-tools REQUIRED CONFIG COMPONENTS filters grpc-interface)
find_package(duna_optimizer REQUIRED CONFIG)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(tst src.cpp)
target_link_libraries(tst duna-cloud-tools::filters  duna::duna_optimizer) # duna-cloud-tools::grpc-interface fails