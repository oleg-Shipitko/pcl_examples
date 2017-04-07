cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iss_keypoints)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (iss_keypoints iss_keypoints.cpp)
target_link_libraries (iss_keypoints  ${PCL_LIBRARIES})