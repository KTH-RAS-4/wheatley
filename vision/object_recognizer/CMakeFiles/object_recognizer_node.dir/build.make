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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/john/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/john/catkin_ws/src

# Include any dependencies generated for this target.
include vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/depend.make

# Include the progress variables for this target.
include vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/progress.make

# Include the compile flags for this target's objects.
include vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/flags.make

vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o: vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/flags.make
vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o: vision/object_recognizer/src/object_recognizer_node.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/john/catkin_ws/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o"
	cd /home/john/catkin_ws/src/vision/object_recognizer && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o -c /home/john/catkin_ws/src/vision/object_recognizer/src/object_recognizer_node.cpp

vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.i"
	cd /home/john/catkin_ws/src/vision/object_recognizer && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/john/catkin_ws/src/vision/object_recognizer/src/object_recognizer_node.cpp > CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.i

vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.s"
	cd /home/john/catkin_ws/src/vision/object_recognizer && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/john/catkin_ws/src/vision/object_recognizer/src/object_recognizer_node.cpp -o CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.s

vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o.requires:
.PHONY : vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o.requires

vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o.provides: vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o.requires
	$(MAKE) -f vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/build.make vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o.provides.build
.PHONY : vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o.provides

vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o.provides.build: vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o

# Object files for target object_recognizer_node
object_recognizer_node_OBJECTS = \
"CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o"

# External object files for target object_recognizer_node
object_recognizer_node_EXTERNAL_OBJECTS =

/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libcv_bridge.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_videostab.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_video.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_superres.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_stitching.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_photo.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_ocl.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_objdetect.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_nonfree.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_ml.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_legacy.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_imgproc.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_highgui.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_gpu.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_flann.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_features2d.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_core.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_contrib.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libopencv_calib3d.so.2.4.9
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libimage_transport.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libmessage_filters.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /usr/lib/libtinyxml.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libclass_loader.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /usr/lib/libPocoFoundation.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /usr/lib/x86_64-linux-gnu/libdl.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libroscpp.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /usr/lib/libboost_signals-mt.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /usr/lib/libboost_filesystem-mt.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/librosconsole.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/librosconsole_log4cxx.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/librosconsole_backend_interface.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /usr/lib/liblog4cxx.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /usr/lib/libboost_regex-mt.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libxmlrpcpp.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libroslib.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libroscpp_serialization.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/librostime.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /usr/lib/libboost_date_time-mt.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /usr/lib/libboost_system-mt.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /usr/lib/libboost_thread-mt.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libcpp_common.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: /opt/ros/hydro/lib/libconsole_bridge.so
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/build.make
/home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node: vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node"
	cd /home/john/catkin_ws/src/vision/object_recognizer && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/object_recognizer_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/build: /home/john/catkin_ws/devel/lib/object_recognizer/object_recognizer_node
.PHONY : vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/build

vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/requires: vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/src/object_recognizer_node.cpp.o.requires
.PHONY : vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/requires

vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/clean:
	cd /home/john/catkin_ws/src/vision/object_recognizer && $(CMAKE_COMMAND) -P CMakeFiles/object_recognizer_node.dir/cmake_clean.cmake
.PHONY : vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/clean

vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/depend:
	cd /home/john/catkin_ws/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/john/catkin_ws/src /home/john/catkin_ws/src/vision/object_recognizer /home/john/catkin_ws/src /home/john/catkin_ws/src/vision/object_recognizer /home/john/catkin_ws/src/vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vision/object_recognizer/CMakeFiles/object_recognizer_node.dir/depend

