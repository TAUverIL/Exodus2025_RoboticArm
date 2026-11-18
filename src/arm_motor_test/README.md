# arm_motor_test

Launch file and ros2_control config for testing a single CubeMars motor.

Note: this folder is not a buildable ROS package (it has no `package.xml`), and its launch file looks for a `controllers.yaml` that is not here. The working single-motor test is in `arm_controller`:

```bash
ros2 launch arm_controller cubemars_test.launch.py
```
