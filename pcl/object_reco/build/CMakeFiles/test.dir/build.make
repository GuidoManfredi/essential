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
CMAKE_SOURCE_DIR = /home/gmanfred/devel/pcl/objectReco

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gmanfred/devel/pcl/objectReco/build

# Include any dependencies generated for this target.
include CMakeFiles/test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test.dir/flags.make

CMakeFiles/test.dir/testbug.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/testbug.cpp.o: ../testbug.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/gmanfred/devel/pcl/objectReco/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test.dir/testbug.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test.dir/testbug.cpp.o -c /home/gmanfred/devel/pcl/objectReco/testbug.cpp

CMakeFiles/test.dir/testbug.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/testbug.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/gmanfred/devel/pcl/objectReco/testbug.cpp > CMakeFiles/test.dir/testbug.cpp.i

CMakeFiles/test.dir/testbug.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/testbug.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/gmanfred/devel/pcl/objectReco/testbug.cpp -o CMakeFiles/test.dir/testbug.cpp.s

CMakeFiles/test.dir/testbug.cpp.o.requires:
.PHONY : CMakeFiles/test.dir/testbug.cpp.o.requires

CMakeFiles/test.dir/testbug.cpp.o.provides: CMakeFiles/test.dir/testbug.cpp.o.requires
	$(MAKE) -f CMakeFiles/test.dir/build.make CMakeFiles/test.dir/testbug.cpp.o.provides.build
.PHONY : CMakeFiles/test.dir/testbug.cpp.o.provides

CMakeFiles/test.dir/testbug.cpp.o.provides.build: CMakeFiles/test.dir/testbug.cpp.o

# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/testbug.cpp.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

test: CMakeFiles/test.dir/testbug.cpp.o
test: /usr/lib/libboost_system-mt.so
test: /usr/lib/libboost_filesystem-mt.so
test: /usr/lib/libboost_thread-mt.so
test: /usr/lib/libboost_date_time-mt.so
test: /usr/lib/libboost_iostreams-mt.so
test: /usr/local/lib/libpcl_common.so
test: /usr/local/lib/libpcl_octree.so
test: /usr/lib/libOpenNI.so
test: /usr/lib/libvtkCommon.so.5.8.0
test: /usr/lib/libvtkRendering.so.5.8.0
test: /usr/lib/libvtkHybrid.so.5.8.0
test: /usr/local/lib/libpcl_io.so
test: /opt/ros/fuerte/lib/libflann_cpp_s.a
test: /usr/local/lib/libpcl_kdtree.so
test: /usr/local/lib/libpcl_search.so
test: /usr/lib/libqhull.so
test: /usr/local/lib/libpcl_surface.so
test: /usr/local/lib/libpcl_sample_consensus.so
test: /usr/local/lib/libpcl_filters.so
test: /usr/local/lib/libpcl_features.so
test: /usr/local/lib/libpcl_geometry.so
test: /usr/local/lib/libpcl_visualization.so
test: /usr/local/lib/libpcl_keypoints.so
test: /usr/local/lib/libpcl_ml.so
test: /usr/local/lib/libpcl_segmentation.so
test: /usr/local/lib/libpcl_registration.so
test: /usr/local/lib/libpcl_recognition.so
test: /usr/local/lib/libpcl_outofcore.so
test: /usr/local/lib/libpcl_cuda_io.so
test: /usr/local/lib/libpcl_cuda_segmentation.so
test: /usr/local/lib/libpcl_cuda_sample_consensus.so
test: /usr/local/lib/libpcl_cuda_features.so
test: /usr/local/lib/libpcl_gpu_containers.so
test: /usr/local/lib/libpcl_gpu_utils.so
test: /usr/local/lib/libpcl_gpu_kinfu_large_scale.so
test: /usr/local/lib/libpcl_gpu_kinfu.so
test: /usr/local/lib/libpcl_gpu_octree.so
test: /usr/local/lib/libpcl_gpu_segmentation.so
test: /usr/local/lib/libpcl_gpu_features.so
test: /usr/local/lib/libpcl_tracking.so
test: /usr/local/lib/libpcl_stereo.so
test: /usr/local/lib/libpcl_apps.so
test: /usr/lib/libvtkParallel.so.5.8.0
test: /usr/lib/libvtkRendering.so.5.8.0
test: /usr/lib/libvtkGraphics.so.5.8.0
test: /usr/lib/libvtkImaging.so.5.8.0
test: /usr/lib/libvtkIO.so.5.8.0
test: /usr/lib/libvtkFiltering.so.5.8.0
test: /usr/lib/libvtkCommon.so.5.8.0
test: /usr/lib/libvtksys.so.5.8.0
test: CMakeFiles/test.dir/build.make
test: CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test.dir/build: test
.PHONY : CMakeFiles/test.dir/build

CMakeFiles/test.dir/requires: CMakeFiles/test.dir/testbug.cpp.o.requires
.PHONY : CMakeFiles/test.dir/requires

CMakeFiles/test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test.dir/clean

CMakeFiles/test.dir/depend:
	cd /home/gmanfred/devel/pcl/objectReco/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gmanfred/devel/pcl/objectReco /home/gmanfred/devel/pcl/objectReco /home/gmanfred/devel/pcl/objectReco/build /home/gmanfred/devel/pcl/objectReco/build /home/gmanfred/devel/pcl/objectReco/build/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test.dir/depend
