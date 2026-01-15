import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory("arm_controller")
    
    # 2. Parse URDF
    xacro_file = os.path.join(pkg_path, "urdf", "dynamixel_test.ros2_control.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    robot_description_content = doc.toxml()
    
    robot_description = {"robot_description": robot_description_content}

    # 3. Path to Controller Config
    controller_config = os.path.join(pkg_path, "config", "controllers.yaml")

    # 4. Define Nodes
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    node_ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="both",
    )

    # Spawns the controller that LISTENS to commands
    node_spawner_forward = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawns the controller that PUBLISHES states (Fixes your empty topic!)
    node_spawner_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_ros2_control,
        node_spawner_forward,
        node_spawner_broadcaster,
    ])