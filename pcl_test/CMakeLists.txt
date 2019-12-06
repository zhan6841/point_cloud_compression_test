cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_compression_test)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_compression_test pcl_test_compression.cpp)
target_link_libraries (pcl_compression_test ${PCL_LIBRARIES})