import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import yaml

def load_yaml(package_name, file_path):
    package_path = FindPackageShare(package_name).find(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # 1. Load Servo Configuration
    servo_yaml = load_yaml("arm_moveit_config", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # 2. Get Robot Description (URDF & SRDF)
    # Note: We reuse the logic from your moveit.launch.py or load strictly here
    # For simplicity, assuming robot_description is available or loaded via xacro here:
    # (You may need to copy the robot_description loading logic from your moveit.launch.py 
    # if this file is run standalone)

    # 3. Servo Node (The core servoing logic)
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            # NOTE: You must pass robot_description and semantic here if running standalone
            # If running included in moveit.launch.py, ensure these are passed down
        ],
        output="screen",
    )

    # 4. Joy Node (Reads the physical Xbox controller)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    # 5. Teleop Twist Joy (Converts Xbox buttons to TwistStamped)
    # We remap the output to the topic Servo listens to
    joy_teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="joy_teleop",
        parameters=[{
            "publish_stamped_twist": True, # Servo needs TwistStamped, not Twist
            "axis_linear.x": 1,            # Left stick vertical
            "axis_linear.y": 0,            # Left stick horizontal
            "axis_angular.yaw": 3,         # Right stick horizontal
            "scale_linear.x": 0.5,
            "scale_linear.y": 0.5,
            "scale_angular.yaw": 0.5,
            "enable_button": 5,            # RB button (usually) to enable movement
            "require_enable_button": True,
            "frame": "base_link",          # Frame the commands are given in
        }],
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