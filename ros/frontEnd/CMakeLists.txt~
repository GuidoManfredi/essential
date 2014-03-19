cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# Go to a release build, because I'm done debugging and I want compiler optimizations
set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE RelWithDebInfo)

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

set(CMAKE_MODULE_PATH /home/gmanfred/devel/libraries/g2o/trunk/cmake_modules/)
find_package (G2O REQUIRED)
find_package (SuiteSparse REQUIRED)

#include_directories(${G2O_INCLUDE_DIR})
include_directories(/home/gmanfred/devel/libraries/g2o/trunk/
                    ${CHOLMOD_INCLUDE_DIR})
                    
set(LIBS ${LIBS} ${G2O_STUFF_LIBRARIES} ${G2O_CORE_LIBRARIES})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(frontEnd src/main.cpp 
								 src/FrontEnd.cpp 
								 src/Frame.cpp
								 src/Pipeline2D.cpp
								 src/Pipeline2DGPU.cpp
								 src/PipelineGeom.cpp
								 src/PoseEstimator.cpp
								 src/Random.cpp
								 src/Timestamp.cpp
								 src/rpnp.cpp)
rosbuild_add_executable(test_rpnp src/test_rpnp.cpp
								  src/rpnp.cpp)
#rosbuild_add_executable(test_kinect src/test_kinect.cpp)								  
target_link_libraries(frontEnd /home/gmanfred/devel/sandbox/SiftGPU/bin/libsiftgpu.so
                               /home/gmanfred/devel/libraries/g2o/trunk/lib/libg2o_backend.so
                               /home/gmanfred/devel/libraries/g2o/trunk/lib/libg2o_interactive.so
                               /home/gmanfred/devel/libraries/g2o/trunk/lib/libg2o_types_slam3d.so
                               /home/gmanfred/devel/libraries/g2o/trunk/lib/libg2o_core.so)
