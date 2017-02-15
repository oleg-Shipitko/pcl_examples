# export OPENNI2_INCLUDE=/usr/include/openni2/
# export OPENNI2_REDIST=/usr/lib/

cmake_minimum_required(VERSION 2.8)
project(objFinder)
find_package(OpenCV REQUIRED)
# set(PCL_DIR "/path/to/PCLConfig.cmake")
find_package(PCL 1.7 REQUIRED)

set(
    CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_HOME_DIRECTORY}/build
    )

add_definitions(
    -std=c++11
    )

include_directories(
    $ENV{OPENNI2_INCLUDE}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_HOME_DIRECTORY}
    )

link_directories(
    $ENV{OPENNI2_REDIST}
    ${PCL_LIBRARY_DIRS}
    )

add_definitions(${PCL_DEFINITIONS})

add_library(PlaneDetector PlaneDetector.cpp)
add_library(TempMatching  TempMatching.cpp)
add_library(Skeleton  Skeleton.cpp)

link_libraries(
    -lOpenNI2
    -lopencv_core 
    -lopencv_highgui
    -lopencv_imgproc 
    -lopencv_imgcodecs
    )

add_executable(
    ${PROJECT_NAME}
    main.cpp
    )

target_link_libraries(${PROJECT_NAME}
    PlaneDetector
    TempMatching
    Skeleton
    opencv_core
    opencv_xfeatures2d
    opencv_features2d
    
    opencv_imgproc
    opencv_highgui
    opencv_ml
    opencv_video
    opencv_videoio
    opencv_imgcodecs
    # ${PCL_COMMON_LIBRARIES} 
    ${PCL_LIBRARIES}
    # pcl_common 
    # pcl_io 
    pcl_kdtree 
    # pcl_octree
    # pcl_on_nurbs
    # pcl_visualization
)
