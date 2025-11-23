"""
Launch file for MoveIt2 with the Exodus2025 robotic arm
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name: str, relative_path: str):
    pkg_share = get_package_share_directory(package_name)
    file_path = os.path.join(pkg_share, relative_path)
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    # Initialize arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # Planning context
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("arm_description"), "urdf", "Arm_MoveIt_Assembly.SLDASM.urdf"]
            ),
            " use_fake_hardware:=",
            use_fake_hardware,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}


    robot_description_semantic_content = Command(
        [
            FindExecutable(name="cat"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("arm_moveit_config"), "config", "Arm_MoveIt_Assembly.SLDASM.srdf"]
            ),
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }
        

    kinematics_yaml = load_yaml("arm_moveit_config", "config/kinematics.yaml")


    # TODO: Add ompl_planning.yaml, pilz_cartesian_limits.yaml, etc.

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        },
    }


    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml("arm_moveit_config", "config/moveit_controllers.yaml")


    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    joint_limits_yaml = load_yaml("arm_moveit_config", "config/joint_limits.yaml")


    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_simple_controllers_yaml,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
        ],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ros2_control node (if using fake hardware, this will be important)
    # TODO: Add controller manager node when controllers are configured

    nodes_to_start = [
        robot_state_publisher,
        run_move_group_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

