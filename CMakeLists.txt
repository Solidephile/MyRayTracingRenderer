cmake_minimum_required(VERSION 3.5)

project(RayTracing)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/bin/debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/bin/release)

add_executable(main src/main.cpp)
target_include_directories(main PRIVATE include)