################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(robot_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
  controller_manager
  joint_state_controller
  robot_state_publisher
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  quadrotor_msgs
)

find_package(gazebo REQUIRED)

find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros
  DEPENDS gazebo
)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(eefixed src/eefixed.cpp include/arm_controller_pub.h include/gripper_controller_pub.h include/wheel_controller_pub.h include/KDL_IK.h)
add_dependencies(eefixed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(eefixed ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(teleop_twist_keyboard src/teleop_twist_keyboard.cpp include/arm_controller_pub.h include/gripper_controller_pub.h include/wheel_controller_pub.h include/KDL_IK.h)
add_dependencies(teleop_twist_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_twist_keyboard ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_executable(twist_to_4wheel_bridge src/twist_to_4wheel.cpp)
# target_link_libraries(twist_to_4wheel_bridge ${catkin_LIBRARIES})
# add_dependencies(twist_to_4wheel_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


################################################################################
# Install
################################################################################


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


install(DIRECTORY launch models rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)