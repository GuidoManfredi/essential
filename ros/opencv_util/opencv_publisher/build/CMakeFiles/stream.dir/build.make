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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher/build

# Include any dependencies generated for this target.
include CMakeFiles/stream.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stream.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stream.dir/flags.make

CMakeFiles/stream.dir/src/stream.cpp.o: CMakeFiles/stream.dir/flags.make
CMakeFiles/stream.dir/src/stream.cpp.o: ../src/stream.cpp
CMakeFiles/stream.dir/src/stream.cpp.o: ../manifest.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/cpp_common/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/rostime/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/roscpp_traits/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/roscpp_serialization/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/genmsg/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/genpy/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/message_runtime/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/std_msgs/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/geometry_msgs/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/sensor_msgs/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/catkin/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/opencv2/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/rosconsole/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/cv_bridge/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/rosgraph_msgs/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/xmlrpcpp/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/roscpp/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/message_filters/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/console_bridge/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/class_loader/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/rospack/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/roslib/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/pluginlib/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/image_transport/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/camera_calibration_parsers/package.xml
CMakeFiles/stream.dir/src/stream.cpp.o: /opt/ros/hydro/share/camera_info_manager/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/stream.dir/src/stream.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/stream.dir/src/stream.cpp.o -c /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher/src/stream.cpp

CMakeFiles/stream.dir/src/stream.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stream.dir/src/stream.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher/src/stream.cpp > CMakeFiles/stream.dir/src/stream.cpp.i

CMakeFiles/stream.dir/src/stream.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stream.dir/src/stream.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher/src/stream.cpp -o CMakeFiles/stream.dir/src/stream.cpp.s

CMakeFiles/stream.dir/src/stream.cpp.o.requires:
.PHONY : CMakeFiles/stream.dir/src/stream.cpp.o.requires

CMakeFiles/stream.dir/src/stream.cpp.o.provides: CMakeFiles/stream.dir/src/stream.cpp.o.requires
	$(MAKE) -f CMakeFiles/stream.dir/build.make CMakeFiles/stream.dir/src/stream.cpp.o.provides.build
.PHONY : CMakeFiles/stream.dir/src/stream.cpp.o.provides

CMakeFiles/stream.dir/src/stream.cpp.o.provides.build: CMakeFiles/stream.dir/src/stream.cpp.o

# Object files for target stream
stream_OBJECTS = \
"CMakeFiles/stream.dir/src/stream.cpp.o"

# External object files for target stream
stream_EXTERNAL_OBJECTS =

../bin/stream: CMakeFiles/stream.dir/src/stream.cpp.o
../bin/stream: CMakeFiles/stream.dir/build.make
../bin/stream: CMakeFiles/stream.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/stream"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stream.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stream.dir/build: ../bin/stream
.PHONY : CMakeFiles/stream.dir/build

CMakeFiles/stream.dir/requires: CMakeFiles/stream.dir/src/stream.cpp.o.requires
.PHONY : CMakeFiles/stream.dir/requires

CMakeFiles/stream.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stream.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stream.dir/clean

CMakeFiles/stream.dir/depend:
	cd /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher/build /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher/build /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_publisher/build/CMakeFiles/stream.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stream.dir/depend
