import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError: 
        return None

def load_yaml(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    
    # --- 1. CONFIGURATION ---
    
    # Load URDF (Robot Description)
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
    robot_description = {"robot_description": robot_description_content}

    # Load SRDF (Semantic Description)
    robot_description_semantic_config = load_file(
        "arm_moveit_config", "config/Arm_MoveIt_Assembly.SLDASM.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Load Kinematics
    kinematics_yaml = load_yaml("arm_moveit_config", "config/kinematics.yaml")
    
    # Load Servo Params
    servo_yaml = load_yaml("arm_moveit_config", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    
    # Load Joystick Params
    joy_config = load_yaml("arm_moveit_config", "config/xbox_teleop.yaml")
    
    # Load Rviz Config
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", "moveit.rviz"]
    )

    # --- 2. NODES ---

    # A. Robot State Publisher (Publishes TF / Robot Model)
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # B. ROS2 Control Node (Simulates the Hardware)
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("arm_controller"),
            "config",
            "controllers.yaml",
        ]
    )
    node_ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # C. Spawners (Start the controllers)
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # D. RViz (Visualization)
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    # E. Servo Node (The Brain)
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
        output="screen",
    )

    # F. Joystick Nodes (The Input)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    joy_teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="joy_teleop",
        parameters=[joy_config],
        remappings=[
            ("/cmd_vel", "/servo_node/delta_twist_cmds"),
        ],
        output="screen",
    )

    # --- 3. EXECUTION FLOW ---
    return LaunchDescription([
        node_robot_state_publisher,
        node_ros2_control,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        node_rviz,
        # Start Servo & Joy AFTER the controllers are ready
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_arm_controller,
                on_exit=[servo_node, joy_node, joy_teleop_node],
            )
        ),
    ])