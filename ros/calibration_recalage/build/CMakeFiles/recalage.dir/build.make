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
CMAKE_SOURCE_DIR = /home/gmanfred/devel/pcl/calibration_recalage

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gmanfred/devel/pcl/calibration_recalage/build

# Include any dependencies generated for this target.
include CMakeFiles/recalage.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/recalage.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/recalage.dir/flags.make

CMakeFiles/recalage.dir/src/recalage.cpp.o: CMakeFiles/recalage.dir/flags.make
CMakeFiles/recalage.dir/src/recalage.cpp.o: ../src/recalage.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/gmanfred/devel/pcl/calibration_recalage/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/recalage.dir/src/recalage.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/recalage.dir/src/recalage.cpp.o -c /home/gmanfred/devel/pcl/calibration_recalage/src/recalage.cpp

CMakeFiles/recalage.dir/src/recalage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/recalage.dir/src/recalage.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/gmanfred/devel/pcl/calibration_recalage/src/recalage.cpp > CMakeFiles/recalage.dir/src/recalage.cpp.i

CMakeFiles/recalage.dir/src/recalage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/recalage.dir/src/recalage.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/gmanfred/devel/pcl/calibration_recalage/src/recalage.cpp -o CMakeFiles/recalage.dir/src/recalage.cpp.s

CMakeFiles/recalage.dir/src/recalage.cpp.o.requires:
.PHONY : CMakeFiles/recalage.dir/src/recalage.cpp.o.requires

CMakeFiles/recalage.dir/src/recalage.cpp.o.provides: CMakeFiles/recalage.dir/src/recalage.cpp.o.requires
	$(MAKE) -f CMakeFiles/recalage.dir/build.make CMakeFiles/recalage.dir/src/recalage.cpp.o.provides.build
.PHONY : CMakeFiles/recalage.dir/src/recalage.cpp.o.provides

CMakeFiles/recalage.dir/src/recalage.cpp.o.provides.build: CMakeFiles/recalage.dir/src/recalage.cpp.o

# Object files for target recalage
recalage_OBJECTS = \
"CMakeFiles/recalage.dir/src/recalage.cpp.o"

# External object files for target recalage
recalage_EXTERNAL_OBJECTS =

recalage: CMakeFiles/recalage.dir/src/recalage.cpp.o
recalage: /usr/lib/libboost_system-mt.so
recalage: /usr/lib/libboost_filesystem-mt.so
recalage: /usr/lib/libboost_thread-mt.so
recalage: /usr/lib/libboost_date_time-mt.so
recalage: /usr/lib/libboost_iostreams-mt.so
recalage: /usr/lib/libboost_mpi-mt.so
recalage: /usr/lib/libboost_serialization-mt.so
recalage: /usr/local/lib/libpcl_common.so
recalage: /usr/local/lib/libpcl_octree.so
recalage: /usr/lib/libvtkCommon.so.5.8.0
recalage: /usr/lib/libvtkRendering.so.5.8.0
recalage: /usr/lib/libvtkHybrid.so.5.8.0
recalage: /usr/local/lib/libpcl_io.so
recalage: /opt/ros/groovy/lib/libflann_cpp_s.a
recalage: /usr/local/lib/libpcl_kdtree.so
recalage: /usr/local/lib/libpcl_search.so
recalage: /usr/lib/libqhull.so
recalage: /usr/local/lib/libpcl_surface.so
recalage: /usr/local/lib/libpcl_sample_consensus.so
recalage: /usr/local/lib/libpcl_filters.so
recalage: /usr/local/lib/libpcl_features.so
recalage: /usr/local/lib/libpcl_visualization.so
recalage: /usr/local/lib/libpcl_keypoints.so
recalage: /usr/local/lib/libpcl_ml.so
recalage: /usr/local/lib/libpcl_segmentation.so
recalage: /usr/local/lib/libpcl_registration.so
recalage: /usr/local/lib/libpcl_recognition.so
recalage: /usr/local/lib/libpcl_outofcore.so
recalage: /usr/local/lib/libpcl_cuda_io.so
recalage: /usr/local/lib/libpcl_cuda_segmentation.so
recalage: /usr/local/lib/libpcl_cuda_sample_consensus.so
recalage: /usr/local/lib/libpcl_cuda_features.so
recalage: /usr/local/lib/libpcl_stereo.so
recalage: /usr/local/lib/libpcl_gpu_containers.so
recalage: /usr/local/lib/libpcl_gpu_utils.so
recalage: /usr/local/lib/libpcl_gpu_kinfu_large_scale.so
recalage: /usr/local/lib/libpcl_gpu_kinfu.so
recalage: /usr/local/lib/libpcl_gpu_octree.so
recalage: /usr/local/lib/libpcl_gpu_segmentation.so
recalage: /usr/local/lib/libpcl_gpu_features.so
recalage: /usr/local/lib/libpcl_tracking.so
recalage: /usr/local/lib/libpcl_apps.so
recalage: /usr/lib/libvtkParallel.so.5.8.0
recalage: /usr/lib/libvtkRendering.so.5.8.0
recalage: /usr/lib/libvtkGraphics.so.5.8.0
recalage: /usr/lib/libvtkImaging.so.5.8.0
recalage: /usr/lib/libvtkIO.so.5.8.0
recalage: /usr/lib/libvtkFiltering.so.5.8.0
recalage: /usr/lib/libvtkCommon.so.5.8.0
recalage: /usr/lib/libvtksys.so.5.8.0
recalage: CMakeFiles/recalage.dir/build.make
recalage: CMakeFiles/recalage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable recalage"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/recalage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/recalage.dir/build: recalage
.PHONY : CMakeFiles/recalage.dir/build

CMakeFiles/recalage.dir/requires: CMakeFiles/recalage.dir/src/recalage.cpp.o.requires
.PHONY : CMakeFiles/recalage.dir/requires

CMakeFiles/recalage.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/recalage.dir/cmake_clean.cmake
.PHONY : CMakeFiles/recalage.dir/clean

CMakeFiles/recalage.dir/depend:
	cd /home/gmanfred/devel/pcl/calibration_recalage/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gmanfred/devel/pcl/calibration_recalage /home/gmanfred/devel/pcl/calibration_recalage /home/gmanfred/devel/pcl/calibration_recalage/build /home/gmanfred/devel/pcl/calibration_recalage/build /home/gmanfred/devel/pcl/calibration_recalage/build/CMakeFiles/recalage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/recalage.dir/depend

