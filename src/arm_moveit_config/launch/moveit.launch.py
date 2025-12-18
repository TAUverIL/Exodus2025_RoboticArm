"""
Launch file for MoveIt2 with the Exodus2025 robotic arm
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name: str, relative_path: str):
    pkg_share = get_package_share_directory(package_name)
    file_path = os.path.join(pkg_share, relative_path)
    try:
        with open(file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_internal_joint_state_publisher",
            default_value="false",
            description="Launch the internal joint_state_publisher (set false if an external one already runs)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_internal_robot_state_publisher",
            default_value="false",
            description="Launch the internal robot_state_publisher (set false if an external one already runs)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_ros2_control",
            default_value="true",
            description="Launch ros2_control (controller_manager + controllers) for the MoveIt robot model",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_servo",
            default_value="false",
            description="Launch Servo node for teleoperation with a joystick",
        )
    )

    # Initialize arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_internal_joint_state_publisher = LaunchConfiguration("launch_internal_joint_state_publisher")
    launch_internal_robot_state_publisher = LaunchConfiguration("launch_internal_robot_state_publisher")
    launch_ros2_control = LaunchConfiguration("launch_ros2_control")
    use_servo = LaunchConfiguration("use_servo")

    # Planning context
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("arm_description"), "urdf", "arm.urdf.xacro"]
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
        

    robot_description_kinematics = {
    "robot_description_kinematics": load_yaml(
        "arm_moveit_config",
        "config/kinematics.yaml"
    )
}


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


    # Joint limits for planning
    robot_description_planning = {
        "robot_description_planning": joint_limits_yaml
    }

    # ros2_control: controller manager + controllers for this robot_description
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("arm_controller"), "config", "controllers.yaml"]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        condition=IfCondition(launch_ros2_control),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=IfCondition(launch_ros2_control),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(launch_ros2_control),
    )

    delay_arm_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_simple_controllers_yaml,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            robot_description_planning,
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
            robot_description_kinematics,
        ],
    )

    # Joint State Publisher (publishes /joint_states for MoveIt and robot_state_publisher)
    joint_state_publisher = Node(
       package="joint_state_publisher",
       executable="joint_state_publisher",
       name="joint_state_publisher",
       output="screen",
       condition=IfCondition(use_fake_hardware),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        condition=IfCondition(launch_internal_robot_state_publisher),
    )

    # Static transform from world to base_link so that the "world" frame exists for RViz/MoveIt
    static_tf_world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )
    # --- SERVO SETUP ---
    # Get parameters for the Servo node
    servo_yaml_content = load_yaml("arm_moveit_config", "config/servo.yaml")
    
    if 'servo_node' in servo_yaml_content and 'ros__parameters' in servo_yaml_content['servo_node']:
        servo_params_flat = servo_yaml_content['servo_node']['ros__parameters']
    else:
        servo_params_flat = servo_yaml_content

    servo_params = {"moveit_servo": servo_params_flat}
    
    # MoveIt Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        output="screen",
        condition=IfCondition(use_servo),
    )

    # Joy Node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        condition=IfCondition(use_servo),
    )

    # Teleop Twist Joy Node
    xbox_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", "xbox_teleop.yaml"]
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[
            xbox_config_file, 
            {"publish_stamped_twist": True},
            {"frame": "base_link"},
        ],
        remappings=[("/cmd_vel", "/servo_twist")],
        condition=IfCondition(use_servo),
    )
    
    # Check if the yaml is nested under servo_node/ros__parameters (typical for param dumps)
    # or if it is flat. We need the flat parameters for the node definition.
    if 'servo_node' in servo_yaml_content and 'ros__parameters' in servo_yaml_content['servo_node']:
        servo_params_flat = servo_yaml_content['servo_node']['ros__parameters']
    else:
        servo_params_flat = servo_yaml_content

    # MoveIt Servo expects parameters under the "moveit_servo" namespace
    servo_params = {"moveit_servo": servo_params_flat}
    
    # MoveIt Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        output="screen",
        condition=IfCondition(use_servo),
    )

    # Joy Node (Driver for XBOX controller)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        condition=IfCondition(use_servo),
    )

    # Teleop Twist Joy Node (Converts Joy -> Twist for Servo)
    # Remaps /cmd_vel to /servo_twist as defined in your servo.yaml
    xbox_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", "xbox_teleop.yaml"]
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[
            xbox_config_file, 
            {"publish_stamped_twist": True},
            {"frame": "base_link"},
        ],
        remappings=[("/cmd_vel", "/servo_twist")],
        condition=IfCondition(use_servo),
    )


    # ros2_control node (if using fake hardware, this will be important)
    # TODO: Add controller manager node when controllers are configured

    nodes_to_start = [
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_after_jsb,
        joint_state_publisher,
        robot_state_publisher,
        run_move_group_node,
        rviz_node,
        static_tf_world_to_base,
        servo_node,
        joy_node,
        teleop_twist_joy_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

