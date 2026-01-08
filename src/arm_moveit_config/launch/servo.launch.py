import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. Get Robot Description (URDF) - Needed for Collision Checking
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("arm_description"), "urdf", "arm.urdf.xacro"]
            ),
            " ",
            "use_fake_hardware:=true", 
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # 2. Get SRDF - Needed for Move Groups
    srdf_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="cat")]), 
            " ",
            PathJoinSubstitution([FindPackageShare("arm_moveit_config"), "config", "Arm_MoveIt_Assembly.SLDASM.srdf"])
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(srdf_content, value_type=str)}

    # 3. YOUR NEW CUSTOM NODE
    servo_node = Node(
        package="arm_moveit_config",
        executable="servo_node",       # Runs your new C++ binary
        name="servo_node",
        parameters=[
            robot_description,         # Pass URDF
            robot_description_semantic # Pass SRDF
        ],
        output="screen",
    )

    # 4. Joy Driver
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"deadzone": 0.05, "autorepeat_rate": 20.0}]
    )

    # 5. Bridge Script
    # (Assuming the script is in src/arm_moveit_config/scripts/joy_to_servo.py)
    joy_to_servo_node = Node(
        package="arm_moveit_config",
        executable="joy_to_servo.py",
        name="joy_to_servo_node",
        output="screen",
    )

    return LaunchDescription([
        servo_node,
        joy_node,
        joy_to_servo_node, 
    ])