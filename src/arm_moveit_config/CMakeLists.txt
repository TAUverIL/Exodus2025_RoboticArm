cmake_minimum_required(VERSION 3.8)
project(arm_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning REQUIRED) 
find_package(tf2_ros REQUIRED)

# 2. DEFINE EXECUTABLE
add_executable(servo_node src/servo_node.cpp)

# 3. LINK DEPENDENCIES (The Robust Way)
# "ament_target_dependencies" automatically finds the right library files
# and include directories for these packages.
ament_target_dependencies(servo_node
  rclcpp
  moveit_servo
  moveit_ros_planning
  tf2_ros
)

# 4. INSTALLATION
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/joy_to_servo.py
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS servo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()