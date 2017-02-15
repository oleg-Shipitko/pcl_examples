cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(edge_detection)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (edge_detection edge_detection.cpp)
target_link_libraries (edge_detection ${PCL_LIBRARIES})