# Arm Description

This package contains the URDF description and visual meshes for the Exodus2025 robotic arm.

## Structure

```
arm_description/
├── urdf/
│   ├── arm.urdf.xacro              # Main robot URDF (placeholder geometry)
│   └── arm.ros2_control.xacro      # ros2_control configuration
├── meshes/
│   └── .gitkeep                     # Directory for STL/DAE mesh files
├── config/
│   └── view_robot.rviz             # RViz configuration
└── launch/
    └── view_robot.launch.py        # Launch file for visualization
```

## Current Status

The package currently contains **placeholder geometry** using simple cylinders and boxes. This is sufficient for:
- Testing the control system
- Developing motion planning
- Visualizing joint movements
- Testing fake hardware

## TODO: Add Actual Robot Geometry

When you're ready to add the actual robot geometry:

### 1. Add Mesh Files

Place your mesh files (STL or DAE format) in the `meshes/` directory:
```
meshes/
├── base_link.stl
├── link1.stl
├── link2.stl
├── link3.stl
├── link4.stl
├── link5.stl
└── tool0.stl (if applicable)
```

### 2. Update URDF

In `urdf/arm.urdf.xacro`, replace the placeholder geometry:

**Current (placeholder):**
```xml
<link name="link1">
  <visual>
    <geometry>
      <cylinder length="0.2" radius="0.03"/>
    </geometry>
    ...
  </visual>
</link>
```

**Update to (with meshes):**
```xml
<link name="link1">
  <visual>
    <geometry>
      <mesh filename="package://arm_description/meshes/link1.stl" scale="0.001 0.001 0.001"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <material name="blue">
      <color rgba="0.0 0.0 0.8 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <mesh filename="package://arm_description/meshes/link1.stl" scale="0.001 0.001 0.001"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </collision>
  <inertial>
    <mass value="0.5"/>  <!-- Update with actual mass -->
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
  </inertial>
</link>
```

### 3. Update Joint Parameters

Ensure joint origins match your actual robot dimensions:
```xml
<joint name="joint2" type="revolute">
  <parent link="link1"/>
  <child link="link2"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>  <!-- Update to match actual dimensions -->
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="15.0" velocity="12.0"/>
</joint>
```

### 4. Add Inertial Properties

For accurate dynamics simulation, add proper inertial properties:
- Mass
- Center of mass
- Inertia tensor

You can use CAD software (SolidWorks, Fusion 360) to export these properties, or tools like `meshlab` to calculate them from STL files.

### 5. Update Collision Geometry

For better performance in motion planning, you may want simplified collision meshes:
```xml
<collision>
  <geometry>
    <!-- Use simplified mesh or primitive shapes for faster collision checking -->
    <cylinder length="0.2" radius="0.04"/>
  </geometry>
</collision>
```

## Visualization

### View the robot in RViz:
```bash
ros2 launch arm_description view_robot.launch.py
```

This will open RViz with a joint state publisher GUI where you can manually move the joints.

### With fake hardware:
```bash
ros2 launch arm_description view_robot.launch.py use_fake_hardware:=true
```

## Testing URDF

Check URDF syntax:
```bash
check_urdf install/arm_description/share/arm_description/urdf/arm.urdf.xacro
```

View URDF structure:
```bash
urdf_to_graphiz install/arm_description/share/arm_description/urdf/arm.urdf.xacro
```

## Integration with ros2_control

The package includes `arm.ros2_control.xacro` which defines the hardware interface. It supports:
- Fake hardware (for testing)
- Real CubeMars hardware (via CAN bus)

The hardware interface is selected via the `use_fake_hardware` argument in launch files.

## Tips

1. **Keep visual and collision geometry separate** - Use detailed meshes for visual, simplified for collision
2. **Use xacro macros** - For repeated elements (like bolts, brackets)
3. **Test incrementally** - Add one link at a time and verify in RViz
4. **Units matter** - ROS uses meters, many CAD exports are in millimeters (use scale factor)
5. **Material properties** - Define colors in a separate materials.xacro file for reusability

