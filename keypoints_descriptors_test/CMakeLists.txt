cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(keypoints_descriptors_test)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (keypoints_descriptors_test keypoints_descriptors_test.cpp)
target_link_libraries (keypoints_descriptors_test  ${PCL_LIBRARIES})