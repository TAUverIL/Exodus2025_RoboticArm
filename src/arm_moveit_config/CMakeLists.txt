cmake_minimum_required(VERSION 3.8)
project(arm_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(servo_node src/servo_node.cpp)
ament_target_dependencies(servo_node
  moveit_ros_planning
  moveit_servo
  rclcpp
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/joy_to_servo.py
    scripts/xbox_joint_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS servo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
