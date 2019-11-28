cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(draco_test_compression)

include_directories(/usr/local/include/draco/)
link_directories(/usr/local/lib/)

add_executable (draco_test_compression draco_test_compression.cpp)
target_link_libraries (draco_test_compression draco dracodec dracoenc)