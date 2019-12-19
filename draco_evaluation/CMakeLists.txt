cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(draco_test_compression)

include_directories(../draco/src/draco)
link_directories(../draco/build)

add_executable (draco_evaluation draco_evaluation.cpp)
target_link_libraries (draco_evaluation draco dracodec dracoenc)