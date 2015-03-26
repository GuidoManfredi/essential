cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(Linemod)
find_package(PCL 1.7.1 REQUIRED COMPONENTS common io recognition segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(linemod_test linemod_test.cpp /
                            LinemodWrap.cpp)
target_link_libraries(linemod_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_RECOGNITION_LIBRARIES})
add_executable(online_grabber online_grabber.cpp /
                              OnlineGrabber.cpp /
                              LinemodWrap.cpp)
target_link_libraries(online_grabber ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_RECOGNITION_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
