# Arm MoveIt Config

MoveIt2 configuration package for the Exodus2025 robotic arm.

## Structure

```
arm_moveit_config/
├── config/
│   ├── arm.srdf                    # Semantic robot description
│   ├── kinematics.yaml             # IK solver configuration
│   ├── joint_limits.yaml           # Joint limits for planning
│   ├── moveit_controllers.yaml    # Controller configuration
│   └── moveit.rviz                 # RViz config for MoveIt
└── launch/
    └── moveit.launch.py            # Main MoveIt launch file
```

## Current Status

The package contains a **basic configuration** that is ready to use but may need tuning:
- Planning group "arm" with all 5 joints
- Basic KDL kinematics solver
- Joint limits matching hardware capabilities
- Simple controller configuration

## Quick Start

### Launch MoveIt with Fake Hardware:
```bash
ros2 launch arm_moveit_config moveit.launch.py use_fake_hardware:=true
```

This will open RViz with the MoveIt Motion Planning plugin where you can:
- Set goal states
- Plan trajectories
- Execute motions (in simulation)

### Launch MoveIt with Real Hardware:
```bash
# First, start the robot hardware
ros2 launch arm_controller arm.launch.py use_fake_hardware:=false

# Then, in another terminal, launch MoveIt
ros2 launch arm_moveit_config moveit.launch.py use_fake_hardware:=false
```

## TODO: Complete Configuration

### 1. SRDF Configuration (`config/arm.srdf`)

**Add Named Poses:**
Define useful arm configurations:
```xml
<group_state name="home" group="arm">
  <joint name="joint1" value="0"/>
  <joint name="joint2" value="0"/>
  <joint name="joint3" value="0"/>
  <joint name="joint4" value="0"/>
  <joint name="joint5" value="0"/>
</group_state>

<group_state name="ready" group="arm">
  <joint name="joint1" value="0"/>
  <joint name="joint2" value="-0.5"/>
  <joint name="joint3" value="0.5"/>
  <joint name="joint4" value="0"/>
  <joint name="joint5" value="0"/>
</group_state>
```

**Update Collision Matrix:**
After adding actual meshes, use MoveIt Setup Assistant to automatically compute collision pairs:
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

**Add End Effector (if applicable):**
```xml
<group name="gripper">
  <link name="gripper_link"/>
  <!-- Add gripper joints if applicable -->
</group>

<end_effector name="gripper" parent_link="tool0" group="gripper"/>
```

### 2. Kinematics Configuration (`config/kinematics.yaml`)

**Try Different Solvers:**

For better performance, you might want to try:
- `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` (more robust)
- `cached_ik_kinematics_plugin/CachedIKKinematicsPlugin` (faster repeated queries)

Install TRAC-IK:
```bash
sudo apt install ros-$ROS_DISTRO-trac-ik-kinematics-plugin
```

Update `kinematics.yaml`:
```yaml
arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  solve_type: Speed  # or Distance, Manipulation
```

### 3. Planning Configuration

**Add OMPL Planning Config:**

Create `config/ompl_planning.yaml`:
```yaml
planning_plugin: ompl_interface/OMPLPlanner
request_adapters: >-
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints
start_state_max_bounds_error: 0.1

arm:
  planner_configs:
    - RRTConnect
    - RRT
    - RRTstar
    - TRRT
    - PRM
    - PRMstar
  projection_evaluator: joints(joint1,joint2,joint3)
  longest_valid_segment_fraction: 0.005
```

**Add Pilz Industrial Motion:**

For Cartesian paths and blending:

Create `config/pilz_cartesian_limits.yaml`:
```yaml
cartesian_limits:
  max_trans_vel: 0.5
  max_trans_acc: 1.0
  max_trans_dec: -1.0
  max_rot_vel: 1.57
```

### 4. Controller Configuration

**Add Trajectory Controller:**

Update `config/moveit_controllers.yaml`:
```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller  # If applicable

  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5

  # Add this if you have a gripper
  gripper_controller:
    type: GripperCommand
    action_ns: gripper_action
    default: true
    joints:
      - gripper_joint
```

### 5. Planning Scene Configuration

**Add workspace bounds:**
```python
# In launch file, add:
workspace_parameters = {
    "workspace_parameters": {
        "header": {"frame_id": "base_link"},
        "min_corner": [-0.5, -0.5, 0.0],
        "max_corner": [0.5, 0.5, 1.0],
    }
}
```

**Add default obstacles:**
Create `config/planning_scene.yaml` for common obstacles like tables, walls, etc.

## Testing

### Test IK Solver:
```bash
ros2 run moveit_kinematics test_kinematics arm
```

### Test Planning:
```python
# Create a simple planning script
import rclpy
from moveit_py import MoveItPy

rclpy.init()
moveit = MoveItPy(node_name="test_planning")
arm = moveit.get_planning_component("arm")

# Plan to named state
arm.set_start_state_to_current_state()
arm.set_goal_state(configuration_name="home")
plan_result = arm.plan()

if plan_result:
    print("Planning successful!")
else:
    print("Planning failed!")
```

### Benchmark Planners:
```bash
ros2 run moveit_ros_benchmarks moveit_run_benchmark \
  --benchmark-config config/benchmark.yaml
```

## Integration with Hardware

The MoveIt configuration works with both fake and real hardware through the `arm_controller` package.

**Controllers Required:**
- `joint_state_broadcaster`: Publishes current joint states
- `joint_trajectory_controller`: Executes planned trajectories (needs to be added to `arm_controller`)

**Add trajectory controller to `arm_controller/config/controllers.yaml`:**
```yaml
arm_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

## Common Issues

1. **"No IK solution found"** - Increase timeout or try different solver
2. **Planning fails** - Check joint limits and collision geometry
3. **Execution fails** - Verify controller configuration matches MoveIt config
4. **Slow planning** - Simplify collision meshes, tune planner parameters

## Resources

- [MoveIt 2 Documentation](https://moveit.picknik.ai/main/index.html)
- [MoveIt 2 Tutorials](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html)
- [OMPL Planner Reference](https://ompl.kavrakilab.org/planners.html)

