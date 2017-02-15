cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(plane_segmentation)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (plane_segmentation plane_segmentation.cpp)
target_link_libraries (plane_segmentation ${PCL_LIBRARIES})