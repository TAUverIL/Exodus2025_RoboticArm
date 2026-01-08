position command:
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [6.28]" -1

launch:
ros2 launch arm_controller dynamixel_test.launch.py