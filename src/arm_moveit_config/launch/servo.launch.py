import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None
    
def load_file(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError: 
        return None

def generate_launch_description():
    # 1. Get Robot Description (URDF)
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

    # 2. Get Robot Semantic Description (SRDF)
    robot_description_semantic_config = load_file(
        "arm_moveit_config", "config/Arm_MoveIt_Assembly.SLDASM.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # 3. Get Kinematics Config (NEW!)
    kinematics_yaml = load_yaml("arm_moveit_config", "config/kinematics.yaml")

    # 4. Get Servo Configuration
    servo_yaml = load_yaml("arm_moveit_config", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # 5. Servo Node 
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml, # <--- Added this line
        ],
        output="screen",
    )

    # 6. Joy Node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    # 7. Teleop Twist Joy
    joy_config = load_yaml("arm_moveit_config", "config/xbox_teleop.yaml")
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

    return LaunchDescription([
        servo_node,
        joy_node,
        joy_teleop_node,
    ])