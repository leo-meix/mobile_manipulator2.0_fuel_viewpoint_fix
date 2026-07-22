cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env 
    src/fuel_frontier_cache.cpp
    src/fuel_frontier_detector.cpp
    src/fuel_frontier_target.cpp
    src/fuel_frontier_target_manager.cpp
    src/fuel_frontier_tracker.cpp
    src/fuel_viewpoint_generator.cpp
    src/grid_map.cpp 
    src/map_adapter.cpp
    src/raycast.cpp
    )
target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )  

add_executable(fuel_frontier_detector_node
    src/fuel_frontier_detector_node.cpp
    )
target_link_libraries(fuel_frontier_detector_node
    plan_env
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(fuel_viewpoint_generator_test
      test/fuel_viewpoint_generator_test.cpp)
  if(TARGET fuel_viewpoint_generator_test)
    target_link_libraries(fuel_viewpoint_generator_test plan_env ${catkin_LIBRARIES})
  endif()
endif()
