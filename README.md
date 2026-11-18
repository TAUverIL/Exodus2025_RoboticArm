# Exodus2025_RoboticArm

A ROS2 control system for a 5-DOF robotic arm using CubeMars AK series motors.

## Where to work on what

| Topic | Folder |
|---|---|
| Joint motors (CubeMars over CAN), driver safety | `src/cubemars_hardware` |
| Gripper motor (Dynamixel) | `src/dynamixel_interface` |
| Arm model (URDF), meshes, motor parameters | `src/arm_description` |
| Xbox controller, MoveIt, joint limits | `src/arm_moveit_config` |
| Controllers and main launch files | `src/arm_controller` |
| Camera and object detection (ZED, YOLO, ArUco) | `src/arm_vision` |
| Single-motor test files (see its README) | `src/arm_motor_test` |

Start every task on a new branch from `main` and merge back through a Pull Request.
The last known working state before the 2026 cleanup is tagged `arm-working-2026-05-26`.
`src/AssemURDF_20-05-26.zip` is a newer SolidWorks export (20.5.2026) that is not wired into the code yet.

## Packages

### `cubemars_hardware`
Hardware interface plugin for ros2_control that communicates with CubeMars AK series motors via CAN bus.
- Real hardware interface for actual CubeMars motors
- Fake hardware interface for testing without physical hardware
- See [cubemars_hardware/README.md](src/cubemars_hardware/README.md) for motor setup details

### `arm_description`
URDF description and meshes for the robotic arm.
- Robot URDF with placeholder geometry (ready for actual meshes)
- ros2_control macro with configurable hardware interface
- Launch files for visualization

### `arm_moveit_config`
MoveIt2 configuration for motion planning and control.
- SRDF with planning groups
- Kinematics configuration
- MoveIt controllers configuration
- Joint limits
- Ready to be filled with actual planning configurations

### `dynamixel_interface`
Hardware interface plugin for the Dynamixel motor that drives the gripper.

### `arm_vision`
ZED camera node: YOLO person detection with distance, ArUco markers, orange-object detection. See [arm_vision/README.md](src/arm_vision/README.md).

### `arm_controller`
Launch files and configurations for bringing up the complete system.
- Main launch file with fake/real hardware switching
- Controller configurations
- See [arm_controller/README.md](src/arm_controller/README.md) for usage instructions

## Quick Start

### Prerequisites
```bash
sudo apt install ros-$ROS_DISTRO-ros2-control \
                 ros-$ROS_DISTRO-ros2-controllers \
                 ros-$ROS_DISTRO-controller-manager \
                 ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-moveit
```

### Build
```bash
cd ~/Exodus2025_RoboticArm
colcon build
source install/setup.bash
```

### Launch with Fake Hardware (Testing)
```bash
ros2 launch arm_controller arm.launch.py use_fake_hardware:=true
```

### Launch with Real Hardware
First, setup your CAN interface:
```bash
sudo ip link set can0 type can bitrate 1000000 // TODO - update the real slcan commands TOMER
sudo ip link set can0 up
```

Then launch:
```bash
ros2 launch arm_controller arm.launch.py use_fake_hardware:=false can_interface:=can0
```

### Visualize Only
```bash
ros2 launch arm_description view_robot.launch.py
```

### MoveIt (Motion Planning)
```bash
ros2 launch arm_moveit_config moveit.launch.py use_fake_hardware:=true
```

## Hardware Setup

The system is configured for 5 CubeMars motors with the following CAN IDs:
- Joint 1: CAN ID 30 (0x1E) - Base rotation
- Joint 2: CAN ID 31 (0x1F) - Shoulder - Cubemars AK80-6
- Joint 3: CAN ID 32 (0x20) - Elbow - Cubemars AK80-6
- Joint 4: CAN ID 33 (0x21) - Wrist rotation
- Joint 5: CAN ID 34 (0x22) - Wrist bend

For motor configuration and CAN setup details, see [cubemars_hardware/README.md](src/cubemars_hardware/README.md).

## Testing

Send position commands:
```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.5, -0.5, 0.0, 0.0]"
```

Monitor joint states:
```bash
ros2 topic echo /joint_states
```

List controllers:
```bash
ros2 control list_controllers
```

## Development Status

✅ Hardware interface (real and fake)  
✅ Basic URDF structure  
✅ ros2_control integration  
✅ Controller configuration  
🚧 Actual robot geometry and meshes (TODO)  
🚧 MoveIt planning scene configuration (TODO)  
🚧 End effector/gripper integration (TODO)

## License

MIT License - See LICENSE file for details
