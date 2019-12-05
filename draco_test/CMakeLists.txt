cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(draco_test_compression)

include_directories(../draco/src/draco)
link_directories(../draco/build)

add_executable (draco_test_compression draco_test_compression.cpp)
target_link_libraries (draco_test_compression draco dracodec dracoenc)