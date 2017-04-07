cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(find_boxes_opencv)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(
    -std=c++11
    )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (find_boxes_opencv find_boxes_opencv.cpp)
target_link_libraries (find_boxes_opencv  
	${PCL_LIBRARIES} 
	opencv_imgproc
    opencv_highgui
    opencv_ml
    opencv_video
    opencv_videoio
    opencv_imgcodecs)