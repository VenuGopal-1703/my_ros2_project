cmake_minimum_required(VERSION 3.8)
project(turtlebot_sinusoidal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare the executable
add_executable(turtlebot_movement src/turtlebot_movement.cpp)

# Link dependencies
ament_target_dependencies(turtlebot_movement
  rclcpp
  geometry_msgs
)

# Install the executable
install(TARGETS
  turtlebot_movement
  DESTINATION lib/turtlebot_sinusoidal)


# Install other files such as launch files or configuration files here if needed

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

