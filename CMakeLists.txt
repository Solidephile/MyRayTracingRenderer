cmake_minimum_required(VERSION 3.5)

project(RayTracing)
aux_source_directory(src DIR_TOOT_SRCS)

add_executable(main ${DIR_TOOT_SRCS})