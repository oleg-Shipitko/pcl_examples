# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer"

# Include any dependencies generated for this target.
include CMakeFiles/point_cloud_viewer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/point_cloud_viewer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/point_cloud_viewer.dir/flags.make

CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o: CMakeFiles/point_cloud_viewer.dir/flags.make
CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o: point_cloud_viewer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer/CMakeFiles" $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o -c "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer/point_cloud_viewer.cpp"

CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer/point_cloud_viewer.cpp" > CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.i

CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer/point_cloud_viewer.cpp" -o CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.s

CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o.requires:
.PHONY : CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o.requires

CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o.provides: CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/point_cloud_viewer.dir/build.make CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o.provides.build
.PHONY : CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o.provides

CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o.provides.build: CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o

# Object files for target point_cloud_viewer
point_cloud_viewer_OBJECTS = \
"CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o"

# External object files for target point_cloud_viewer
point_cloud_viewer_EXTERNAL_OBJECTS =

point_cloud_viewer: CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o
point_cloud_viewer: CMakeFiles/point_cloud_viewer.dir/build.make
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_system.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libpthread.so
point_cloud_viewer: /usr/lib/libpcl_common.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
point_cloud_viewer: /usr/lib/libpcl_kdtree.so
point_cloud_viewer: /usr/lib/libpcl_octree.so
point_cloud_viewer: /usr/lib/libpcl_search.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libqhull.so
point_cloud_viewer: /usr/lib/libpcl_surface.so
point_cloud_viewer: /usr/lib/libpcl_sample_consensus.so
point_cloud_viewer: /usr/lib/libOpenNI.so
point_cloud_viewer: /usr/lib/libOpenNI2.so
point_cloud_viewer: /usr/lib/libvtkCommon.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkFiltering.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkImaging.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkGraphics.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkGenericFiltering.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkIO.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkRendering.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkVolumeRendering.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkHybrid.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkWidgets.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkParallel.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkInfovis.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkGeovis.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkViews.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkCharts.so.5.8.0
point_cloud_viewer: /usr/lib/libpcl_io.so
point_cloud_viewer: /usr/lib/libpcl_filters.so
point_cloud_viewer: /usr/lib/libpcl_features.so
point_cloud_viewer: /usr/lib/libpcl_keypoints.so
point_cloud_viewer: /usr/lib/libpcl_registration.so
point_cloud_viewer: /usr/lib/libpcl_segmentation.so
point_cloud_viewer: /usr/lib/libpcl_recognition.so
point_cloud_viewer: /usr/lib/libpcl_visualization.so
point_cloud_viewer: /usr/lib/libpcl_people.so
point_cloud_viewer: /usr/lib/libpcl_outofcore.so
point_cloud_viewer: /usr/lib/libpcl_tracking.so
point_cloud_viewer: /usr/lib/libpcl_apps.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_system.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libpthread.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libqhull.so
point_cloud_viewer: /usr/lib/libOpenNI.so
point_cloud_viewer: /usr/lib/libOpenNI2.so
point_cloud_viewer: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
point_cloud_viewer: /usr/lib/libvtkCommon.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkFiltering.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkImaging.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkGraphics.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkGenericFiltering.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkIO.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkRendering.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkVolumeRendering.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkHybrid.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkWidgets.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkParallel.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkInfovis.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkGeovis.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkViews.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkCharts.so.5.8.0
point_cloud_viewer: /usr/lib/libpcl_common.so
point_cloud_viewer: /usr/lib/libpcl_kdtree.so
point_cloud_viewer: /usr/lib/libpcl_octree.so
point_cloud_viewer: /usr/lib/libpcl_search.so
point_cloud_viewer: /usr/lib/libpcl_surface.so
point_cloud_viewer: /usr/lib/libpcl_sample_consensus.so
point_cloud_viewer: /usr/lib/libpcl_io.so
point_cloud_viewer: /usr/lib/libpcl_filters.so
point_cloud_viewer: /usr/lib/libpcl_features.so
point_cloud_viewer: /usr/lib/libpcl_keypoints.so
point_cloud_viewer: /usr/lib/libpcl_registration.so
point_cloud_viewer: /usr/lib/libpcl_segmentation.so
point_cloud_viewer: /usr/lib/libpcl_recognition.so
point_cloud_viewer: /usr/lib/libpcl_visualization.so
point_cloud_viewer: /usr/lib/libpcl_people.so
point_cloud_viewer: /usr/lib/libpcl_outofcore.so
point_cloud_viewer: /usr/lib/libpcl_tracking.so
point_cloud_viewer: /usr/lib/libpcl_apps.so
point_cloud_viewer: /usr/lib/libvtkViews.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkInfovis.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkWidgets.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkVolumeRendering.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkHybrid.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkParallel.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkRendering.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkImaging.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkGraphics.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkIO.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkFiltering.so.5.8.0
point_cloud_viewer: /usr/lib/libvtkCommon.so.5.8.0
point_cloud_viewer: /usr/lib/libvtksys.so.5.8.0
point_cloud_viewer: CMakeFiles/point_cloud_viewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable point_cloud_viewer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/point_cloud_viewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/point_cloud_viewer.dir/build: point_cloud_viewer
.PHONY : CMakeFiles/point_cloud_viewer.dir/build

CMakeFiles/point_cloud_viewer.dir/requires: CMakeFiles/point_cloud_viewer.dir/point_cloud_viewer.cpp.o.requires
.PHONY : CMakeFiles/point_cloud_viewer.dir/requires

CMakeFiles/point_cloud_viewer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/point_cloud_viewer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/point_cloud_viewer.dir/clean

CMakeFiles/point_cloud_viewer.dir/depend:
	cd "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer" "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer" "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer" "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer" "/home/oleg/Desktop/Shipitko Oleg/pcl_workspace/point_cloud_viewer/CMakeFiles/point_cloud_viewer.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/point_cloud_viewer.dir/depend

