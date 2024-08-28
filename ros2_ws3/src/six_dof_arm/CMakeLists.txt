cmake_minimum_required(VERSION 3.5)
project(six_dof_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_robot_interaction REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fcl REQUIRED)
find_package(moveit_msgs REQUIRED)

include_directories(
  include
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_ros_robot_interaction_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${fcl_INCLUDE_DIRS}
)

add_executable(arm_movement src/arm_movement.cpp)

ament_target_dependencies(arm_movement
  rclcpp
  gazebo_ros
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  moveit_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  fcl
)

install(TARGETS
  arm_movement
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
