"""Launch the Xbox joint-servo bridge for the Exodus2025 arm.

Starts the `joy` driver and the Python `xbox_joint_servo` node, which streams
JointTrajectory messages to the existing arm_controller. Use alongside
`moveit.launch.py` (or any launch that already brings up ros2_control).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_dir = os.path.join(
        get_package_share_directory("arm_moveit_config"), "config")
    default_servo_config = os.path.join(config_dir, "xbox_servo.yaml")

    declared = [
        DeclareLaunchArgument(
            "joy_device",
            default_value="/dev/input/js0",
            description="Path to the Xbox controller device.",
        ),
        DeclareLaunchArgument(
            "servo_config",
            default_value=default_servo_config,
            description="YAML file with xbox_joint_servo parameters.",
        ),
        DeclareLaunchArgument(
            "deadzone",
            default_value="0.05",
            description="joy_node deadzone (the servo applies its own deadband on top).",
        ),
    ]

    joy_device = LaunchConfiguration("joy_device")
    servo_config = LaunchConfiguration("servo_config")
    deadzone = LaunchConfiguration("deadzone")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "device_id": 0,
            "device_name": "",
            "deadzone": deadzone,
            "autorepeat_rate": 30.0,
            "sticky_buttons": False,
            "coalesce_interval_ms": 20,
        }],
    )

    xbox_servo_node = Node(
        package="arm_moveit_config",
        executable="xbox_joint_servo.py",
        name="xbox_joint_servo",
        output="screen",
        parameters=[servo_config],
    )

    return LaunchDescription(declared + [joy_node, xbox_servo_node])
