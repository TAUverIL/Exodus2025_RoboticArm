# Arm Controller

This package contains launch files and controller configuration for bringing up the Exodus2025 robotic arm.

## Usage

### Launch with Fake Hardware (for testing without real motors)

```bash
ros2 launch arm_controller arm.launch.py use_fake_hardware:=true
```

This is useful for:
- Testing controllers without physical hardware
- Visualizing the robot in RViz
- Developing motion planning algorithms
- Testing your software stack

### Launch with Real Hardware (CubeMars motors via CAN)

First, make sure your CAN interface is up:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Then launch with real hardware:

```bash
ros2 launch arm_controller arm.launch.py use_fake_hardware:=false can_interface:=can0
```

### Launch Options

- `use_fake_hardware`: Use fake hardware interface (default: false)
- `can_interface`: CAN interface name (default: slcan0) - only used with real hardware
- `use_rviz`: Launch RViz for visualization (default: true)
- `controllers_file`: YAML file with controllers configuration (default: controllers.yaml)

### Examples

**Test with fake hardware and no RViz:**
```bash
ros2 launch arm_controller arm.launch.py use_fake_hardware:=true use_rviz:=false
```

**Use real hardware with slcan0 interface:**
```bash
ros2 launch arm_controller arm.launch.py use_fake_hardware:=false can_interface:=slcan0
```

**Use the old test launch file (deprecated, kept for reference):**
```bash
ros2 launch arm_controller cubemars_test.launch.py
```

## Controllers

The default controller configuration includes:
- `joint_state_broadcaster`: Publishes joint states to `/joint_states` topic
- `forward_position_controller`: Accepts position commands for all joints

## Testing Commands

Send a position command to all joints:

```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.5, -0.5, 0.0, 0.0]"
```

Monitor joint states:

```bash
ros2 topic echo /joint_states
```

List available controllers:

```bash
ros2 control list_controllers
```

List hardware interfaces:

```bash
ros2 control list_hardware_interfaces
```

