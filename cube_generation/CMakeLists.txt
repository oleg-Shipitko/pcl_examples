cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cube_detection)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cube_detection cube_detection.cpp)
target_link_libraries (cube_detection ${PCL_LIBRARIES})