#!/usr/bin/env python3
"""Xbox controller -> JointTrajectory bridge for the Exodus2025 arm.

Replaces the C++ moveit_servo node. Each joint is mapped to a dedicated
Xbox axis (or D-pad). A deadman button must be held for motion. Joint
positions are integrated from the latest /joint_states and streamed as
JointTrajectory messages directly to the position-based arm_controller,
which is what the existing controller stack actually accepts.

A/B toggle the gripper between configured open/closed positions.
"""

import math
from typing import Dict, List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class XboxJointServo(Node):
    def __init__(self) -> None:
        super().__init__("xbox_joint_servo")

        self.declare_parameter("joint_names", [
            "Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5",
        ])
        # Per-joint Xbox source. Use "axis:<index>" or "dpad:<index>" or
        # "trigger:<index>" or "buttons:<plus>,<minus>". One entry per joint.
        # Default mapping (Xbox/Linux joy layout):
        #   Joint_1: left stick X  (axis 0)
        #   Joint_2: left stick Y  (axis 1, inverted -> push up = +)
        #   Joint_3: right stick Y (axis 4, inverted)
        #   Joint_4: right stick X (axis 3)
        #   Joint_5: triggers (RT - LT)
        self.declare_parameter("joint_sources", [
            "axis:0",
            "axis:1:invert",
            "axis:4:invert",
            "axis:3",
            "trigger:rt-lt",
        ])
        self.declare_parameter("joint_velocity_scales",
                               [1.2, 1.2, 1.0, 1.5, 1.8])  # rad/s at full deflection
        self.declare_parameter("joint_lower_limits",
                               [-3.14, -1.57, -2.5, -3.14, -3.14])
        self.declare_parameter("joint_upper_limits",
                               [3.14, 1.57, 2.5, 3.14, 3.14])

        self.declare_parameter("deadman_button", 5)   # RB
        self.declare_parameter("turbo_button", 4)     # LB - 2x speed when held
        self.declare_parameter("turbo_multiplier", 2.0)
        self.declare_parameter("deadband", 0.12)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("command_duration", 0.12)  # seconds for each traj point

        # Gripper
        self.declare_parameter("gripper_topic", "/gripper_controller/joint_trajectory")
        self.declare_parameter("gripper_joint", "Finger_Left_Joint")
        self.declare_parameter("gripper_open", 0.08)
        self.declare_parameter("gripper_closed", 0.0)
        self.declare_parameter("gripper_open_button", 0)   # A
        self.declare_parameter("gripper_close_button", 1)  # B
        self.declare_parameter("gripper_command_duration", 0.8)

        self.declare_parameter("arm_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("joint_state_topic", "/joint_states")

        self.joint_names: List[str] = list(
            self.get_parameter("joint_names").value)
        self.joint_sources: List[str] = list(
            self.get_parameter("joint_sources").value)
        self.joint_velocity_scales: List[float] = list(
            self.get_parameter("joint_velocity_scales").value)
        self.joint_lower: List[float] = list(
            self.get_parameter("joint_lower_limits").value)
        self.joint_upper: List[float] = list(
            self.get_parameter("joint_upper_limits").value)

        if not (len(self.joint_names) == len(self.joint_sources)
                == len(self.joint_velocity_scales)
                == len(self.joint_lower) == len(self.joint_upper)):
            raise RuntimeError(
                "joint_names, joint_sources, joint_velocity_scales, joint_lower_limits "
                "and joint_upper_limits must all have the same length")

        self.deadman_button = int(self.get_parameter("deadman_button").value)
        self.turbo_button = int(self.get_parameter("turbo_button").value)
        self.turbo_multiplier = float(
            self.get_parameter("turbo_multiplier").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.command_duration = float(
            self.get_parameter("command_duration").value)

        self.gripper_topic = self.get_parameter("gripper_topic").value
        self.gripper_joint = self.get_parameter("gripper_joint").value
        self.gripper_open = float(self.get_parameter("gripper_open").value)
        self.gripper_closed = float(self.get_parameter("gripper_closed").value)
        self.gripper_open_button = int(
            self.get_parameter("gripper_open_button").value)
        self.gripper_close_button = int(
            self.get_parameter("gripper_close_button").value)
        self.gripper_command_duration = float(
            self.get_parameter("gripper_command_duration").value)

        arm_topic = self.get_parameter("arm_topic").value
        joint_state_topic = self.get_parameter("joint_state_topic").value

        self.arm_pub = self.create_publisher(JointTrajectory, arm_topic, 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, self.gripper_topic, 10)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self.create_subscription(
            JointState, joint_state_topic, self._js_cb, 50)

        self.latest_joy: Optional[Joy] = None
        self.prev_buttons: List[int] = []
        self.current_positions: Dict[str, float] = {}
        # Commanded positions are integrated server-side so that holding a stick
        # keeps moving even if /joint_states updates slowly. They snap to the
        # latest measurement whenever the deadman is released.
        self.commanded_positions: Dict[str, float] = {}
        self.have_state = False
        self.last_tick_time = self.get_clock().now()
        self.warned_axes = False

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 1.0), self._tick)

        self.get_logger().info(
            f"xbox_joint_servo ready. Deadman=button {self.deadman_button} "
            f"(hold RB to move). Joints: {self.joint_names}")
        for name, src, scl in zip(self.joint_names, self.joint_sources,
                                  self.joint_velocity_scales):
            self.get_logger().info(f"  {name}  <- {src}  scale={scl:.2f} rad/s")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _joy_cb(self, msg: Joy) -> None:
        self.latest_joy = msg

    def _js_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.current_positions[name] = pos
        self.have_state = all(
            n in self.current_positions for n in self.joint_names)

    # ------------------------------------------------------------------
    # Source decoding
    # ------------------------------------------------------------------
    def _source_value(self, source: str, joy: Joy) -> float:
        """Decode a joint_sources entry against the latest Joy message.

        Returns a value in roughly [-1, 1] before deadband.
        """
        parts = source.split(":")
        kind = parts[0].strip().lower()

        if kind == "axis":
            idx = int(parts[1])
            if idx >= len(joy.axes):
                return 0.0
            value = joy.axes[idx]
            if len(parts) >= 3 and parts[2].strip().lower() == "invert":
                value = -value
            return float(value)

        if kind == "dpad":
            # Treated like an axis (-1, 0, +1) but kept distinct in case we want
            # to add discrete behavior later.
            idx = int(parts[1])
            if idx >= len(joy.axes):
                return 0.0
            value = joy.axes[idx]
            if len(parts) >= 3 and parts[2].strip().lower() == "invert":
                value = -value
            return float(value)

        if kind == "trigger":
            mode = parts[1].strip().lower() if len(parts) > 1 else "rt-lt"
            # Linux joy: LT = axis 2, RT = axis 5, idle = +1, fully pressed = -1.
            if len(joy.axes) <= 5:
                return 0.0
            lt = (1.0 - joy.axes[2]) * 0.5
            rt = (1.0 - joy.axes[5]) * 0.5
            if mode == "rt-lt":
                return rt - lt
            if mode == "lt-rt":
                return lt - rt
            return 0.0

        if kind == "buttons":
            # buttons:<plus>,<minus> -> +1 when plus pressed, -1 when minus pressed
            try:
                plus_s, minus_s = parts[1].split(",")
                plus = int(plus_s)
                minus = int(minus_s)
            except (ValueError, IndexError):
                return 0.0
            p = (joy.buttons[plus] if plus < len(joy.buttons) else 0)
            m = (joy.buttons[minus] if minus < len(joy.buttons) else 0)
            return float(p - m)

        return 0.0

    def _apply_deadband(self, value: float) -> float:
        return value if abs(value) >= self.deadband else 0.0

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_tick_time).nanoseconds * 1e-9
        self.last_tick_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.publish_rate

        joy = self.latest_joy
        if joy is None:
            return

        # Sanity warn if axes layout looks wrong (Linux Xbox joy has 8 axes).
        if len(joy.axes) < 6 and not self.warned_axes:
            self.get_logger().warn(
                "Joy message has fewer than 6 axes; check that joy driver is "
                "configured for an Xbox-style controller.")
            self.warned_axes = True

        self._handle_gripper_buttons(joy)

        prev = self.prev_buttons
        self.prev_buttons = list(joy.buttons)

        deadman_pressed = (
            self.deadman_button < len(joy.buttons)
            and joy.buttons[self.deadman_button])

        if not deadman_pressed:
            # Sync command state to measurement so the next press starts fresh.
            if self.have_state:
                for n in self.joint_names:
                    self.commanded_positions[n] = self.current_positions[n]
            return

        if not self.have_state:
            self.get_logger().warn_once(
                "Waiting for /joint_states before commanding the arm...")
            return

        turbo = (self.turbo_button < len(joy.buttons)
                 and joy.buttons[self.turbo_button])
        speed_mult = self.turbo_multiplier if turbo else 1.0

        any_motion = False
        target_positions: List[float] = []
        for name, source, scale, lo, hi in zip(
                self.joint_names, self.joint_sources,
                self.joint_velocity_scales, self.joint_lower, self.joint_upper):

            raw = self._apply_deadband(self._source_value(source, joy))
            base = self.commanded_positions.get(
                name, self.current_positions.get(name, 0.0))
            if raw != 0.0:
                delta = raw * scale * speed_mult * dt
                new_pos = _clamp(base + delta, lo, hi)
                if not math.isclose(new_pos, base):
                    any_motion = True
                self.commanded_positions[name] = new_pos
            else:
                # Hold position when stick is centered.
                self.commanded_positions[name] = base
            target_positions.append(self.commanded_positions[name])

        if not any_motion:
            # Nothing to do; avoid spamming identical setpoints.
            return

        traj = JointTrajectory()
        traj.header.stamp = now.to_msg()
        traj.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = target_positions
        duration = Duration(seconds=self.command_duration)
        point.time_from_start = duration.to_msg()
        traj.points = [point]
        self.arm_pub.publish(traj)

    # ------------------------------------------------------------------
    # Gripper
    # ------------------------------------------------------------------
    def _handle_gripper_buttons(self, joy: Joy) -> None:
        if not joy.buttons:
            return

        prev = self.prev_buttons

        def rising_edge(idx: int) -> bool:
            if idx < 0 or idx >= len(joy.buttons):
                return False
            now = joy.buttons[idx]
            was = prev[idx] if idx < len(prev) else 0
            return bool(now) and not bool(was)

        if rising_edge(self.gripper_open_button):
            self._send_gripper(self.gripper_open)
        elif rising_edge(self.gripper_close_button):
            self._send_gripper(self.gripper_closed)

    def _send_gripper(self, position: float) -> None:
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [self.gripper_joint]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(
            seconds=self.gripper_command_duration).to_msg()
        traj.points = [point]
        self.gripper_pub.publish(traj)
        self.get_logger().info(f"Gripper -> {position:.3f}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XboxJointServo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
