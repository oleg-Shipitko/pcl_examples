cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(tuto-pairwise)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (poin_cloud_stiching point_cloud_stiching.cpp)
target_link_libraries (poin_cloud_stiching ${PCL_LIBRARIES})
