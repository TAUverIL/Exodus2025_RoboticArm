import os
import re

files = [
    'src/arm_description/urdf/arm.ros2_control.xacro',
    'src/arm_controller/config/controllers.yaml',
    'src/arm_moveit_config/config/moveit_controllers.yaml',
    'src/arm_moveit_config/config/joint_limits.yaml',
    'src/arm_moveit_config/config/Arm_MoveIt_Assembly.SLDASM.srdf'
]

for file_path in files:
    if not os.path.exists(file_path):
        continue
    with open(file_path, 'r') as f:
        content = f.read()

    # Replace Joint1 -> Joint_1
    for i in range(1, 6):
        content = re.sub(rf'\bJoint{i}\b', f'Joint_{i}', content)
        content = re.sub(rf'\bjoint{i}\b', f'Joint_{i}', content) # Fix any lowercase joint1
        
    # Replace link1 -> Link_1
    for i in range(1, 5):
        content = re.sub(rf'\blink{i}\b', f'Link_{i}', content)

    # In SRDF, replace End_effector with Link_5
    content = re.sub(r'\bEnd_effector\b', 'Link_5', content)

    with open(file_path, 'w') as f:
        f.write(content)

print("Replacement complete.")
