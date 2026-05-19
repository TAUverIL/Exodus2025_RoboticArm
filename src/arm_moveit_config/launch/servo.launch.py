from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('arm_moveit_config'),
        'config'
    )
    servo_config = os.path.join(config_dir, 'servo.yaml')

    return LaunchDescription([
        Node(
            package='moveit_servo',
            executable='servo_node_main',
            name='servo_node',
            output='screen',
            parameters=[
                servo_config,
                {"move_group_namespace": "/arm"}
            ]
        )
    ])
