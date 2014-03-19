cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(reco_obb src/reco_obb.cpp src/BoundingBoxer.cpp src/gdiam.cpp src/Reco.cpp)
#rosbuild_add_executable(train_obb src/train_obb.cpp src/BoundingBoxer.cpp src/gdiam.cpp src/TrainOBB.cpp)
rosbuild_add_executable(obb_test src/obb_test.cpp src/OrientedBoundingBox.cpp)
target_link_libraries(obb_test ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkHybrid.so)
#rosbuild_add_executable(clean_training src/clean.cpp src/TrainOBB.cpp)
#target_link_libraries(example ${PROJECT_NAME})
