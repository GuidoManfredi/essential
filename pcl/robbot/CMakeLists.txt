cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(ROBBOT)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

add_executable(test test.cpp TestDistances.cpp Reco.cpp)
target_link_libraries(test ${PCL_LIBRARIES})

#add_executable(reco reco.cpp Reco.cpp)
#target_link_libraries(reco ${PCL_LIBRARIES})

#add_executable(train train.cpp Trainer.cpp TrainerColor.cpp string_tools.cpp)
#target_link_libraries(train ${PCL_LIBRARIES})

#add_executable(compute_obbs compute_obbs.cpp OBBComputer.cpp PlanDetector.cpp gdiam.cpp string_tools.cpp)
#target_link_libraries(compute_obbs ${PCL_LIBRARIES})

#add_executable(cleaner clean.cpp Clean.cpp string_tools.cpp)
#target_link_libraries(cleaner ${PCL_LIBRARIES})
