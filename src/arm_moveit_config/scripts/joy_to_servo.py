#!/usr/bin/env python3

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_LT = 2
AXIS_RIGHT_X = 3
AXIS_RIGHT_Y = 4
AXIS_RT = 5
AXIS_DPAD_X = 6
AXIS_DPAD_Y = 7

BTN_A = 0
BTN_B = 1
BTN_RB = 5


class JoyToServo(Node):
    def __init__(self):
        super().__init__("joy_to_servo_node")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("enable_button", BTN_RB)
        self.declare_parameter("deadband", 0.12)
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("joint_scales", [1.0, 0.6])
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("joint_topic", "/servo_node/delta_joint_cmds")
        self.declare_parameter("gripper_topic", "/gripper_controller/joint_trajectory")
        self.declare_parameter("joint_names", ["Joint_1", "Joint_2"])
        self.declare_parameter("gripper_joint_name", "Finger_Left_Joint")
        self.declare_parameter("gripper_open_position", 0.08)
        self.declare_parameter("gripper_closed_position", 0.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.enable_button = self.get_parameter("enable_button").value
        self.deadband = self.get_parameter("deadband").value
        self.linear_scale = self.get_parameter("linear_scale").value
        self.angular_scale = self.get_parameter("angular_scale").value
        self.joint_scales = list(self.get_parameter("joint_scales").value)
        publish_rate = self.get_parameter("publish_rate").value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.gripper_joint_name = self.get_parameter("gripper_joint_name").value
        self.gripper_open_position = self.get_parameter("gripper_open_position").value
        self.gripper_closed_position = self.get_parameter("gripper_closed_position").value

        twist_topic = self.get_parameter("twist_topic").value
        joint_topic = self.get_parameter("joint_topic").value
        gripper_topic = self.get_parameter("gripper_topic").value

        self.twist_pub = self.create_publisher(TwistStamped, twist_topic, 10)
        self.joint_pub = self.create_publisher(JointJog, joint_topic, 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, gripper_topic, 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.latest_joy = None
        self.previous_buttons = []
        self.warned_bad_axes = False
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_servo_command)

        self.get_logger().info(
            f"Xbox servo bridge ready. Hold button {self.enable_button} to move."
        )

    def joy_callback(self, msg):
        self.latest_joy = msg

    def apply_deadband(self, value):
        return value if abs(value) >= self.deadband else 0.0

    def publish_servo_command(self):
        msg = self.latest_joy
        if msg is None:
            return

        if len(msg.axes) < 8:
            if not self.warned_bad_axes:
                self.get_logger().warn(
                    "Joy message has fewer than 8 axes; expected an Xbox-style layout."
                )
                self.warned_bad_axes = True
            return

        self.handle_gripper_buttons(msg)

        if len(msg.buttons) <= self.enable_button or not msg.buttons[self.enable_button]:
            self.previous_buttons = list(msg.buttons)
            return

        timestamp = self.get_clock().now().to_msg()

        dpad_x = self.apply_deadband(msg.axes[AXIS_DPAD_X])
        dpad_y = self.apply_deadband(msg.axes[AXIS_DPAD_Y])
        if dpad_x != 0.0 or dpad_y != 0.0:
            self.publish_joint_command(timestamp, dpad_x, dpad_y)
            self.previous_buttons = list(msg.buttons)
            return

        linear_x = self.apply_deadband(msg.axes[AXIS_LEFT_Y])
        linear_y = self.apply_deadband(msg.axes[AXIS_LEFT_X])
        angular_y = self.apply_deadband(msg.axes[AXIS_RIGHT_Y])
        angular_z = self.apply_deadband(msg.axes[AXIS_RIGHT_X])

        lt = (1.0 - msg.axes[AXIS_LT]) / 2.0
        rt = (1.0 - msg.axes[AXIS_RT]) / 2.0
        linear_z = self.apply_deadband(rt - lt)

        if (
            linear_x == 0.0 and linear_y == 0.0 and linear_z == 0.0 and
            angular_y == 0.0 and angular_z == 0.0
        ):
            self.previous_buttons = list(msg.buttons)
            return

        twist = TwistStamped()
        twist.header.stamp = timestamp
        twist.header.frame_id = self.frame_id
        twist.twist.linear.x = linear_x * self.linear_scale
        twist.twist.linear.y = linear_y * self.linear_scale
        twist.twist.linear.z = linear_z * self.linear_scale
        twist.twist.angular.y = angular_y * self.angular_scale
        twist.twist.angular.z = angular_z * self.angular_scale
        self.twist_pub.publish(twist)
        self.previous_buttons = list(msg.buttons)

    def publish_joint_command(self, timestamp, dpad_x, dpad_y):
        joint_msg = JointJog()
        joint_msg.header.stamp = timestamp
        joint_msg.header.frame_id = self.frame_id
        joint_msg.joint_names = self.joint_names
        joint_scales = (self.joint_scales + [0.35, 0.35])[:2]
        joint_msg.velocities = [
            dpad_x * joint_scales[0],
            dpad_y * joint_scales[1],
        ]
        self.joint_pub.publish(joint_msg)

    def handle_gripper_buttons(self, msg):
        if len(msg.buttons) <= max(BTN_A, BTN_B):
            return

        previous_a = len(self.previous_buttons) > BTN_A and self.previous_buttons[BTN_A]
        previous_b = len(self.previous_buttons) > BTN_B and self.previous_buttons[BTN_B]

        if msg.buttons[BTN_A] and not previous_a:
            self.publish_gripper_command(self.gripper_open_position)
        elif msg.buttons[BTN_B] and not previous_b:
            self.publish_gripper_command(self.gripper_closed_position)

    def publish_gripper_command(self, position):
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = [self.gripper_joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = 1
        trajectory.points = [point]

        self.gripper_pub.publish(trajectory)
        self.get_logger().info(f"Gripper command: {position:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
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
