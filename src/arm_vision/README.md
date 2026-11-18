# arm_vision

Camera and object detection for the arm (ZED stereo camera).

- `zed_vs.py` — ROS 2 node `zed_camera_node`: detects people with YOLO and publishes their distance, and runs the ArUco and orange-object detectors.
- `ArucoDetector.py` — ArUco marker detection.
- `orange_detector.py` — colour-mask detection of orange objects (used to find the probes).
- `models/yolov8n.pt` — the YOLO model. Installed with the package and loaded from there; override with the `model_path` parameter.

Run:
```bash
colcon build --packages-select arm_vision
ros2 run arm_vision zed_camera_node
```
