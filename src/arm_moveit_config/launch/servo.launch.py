from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_servo',
            executable='servo_node',
            name='servo_node',
            output='screen',
            parameters=[
                "config/servo.yaml",
                {"move_group_namespace": "/arm"}
            ]
        )
    ])
