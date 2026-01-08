#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
import time

class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo_node')
        
        # --- CONFIGURATION ---
        self.frame_id = 'base_link'
        self.enable_button = 5  # RB (Right Bumper) must be held to move
        
        # Publishers (Must match servo.yaml topics)
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        
        # Subscriber
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.get_logger().info("JoyToServo Bridge Initialized. Hold RB (Btn 5) to move.")

    def joy_callback(self, msg):
        # Safety: Deadman switch (RB)
        if not msg.buttons[self.enable_button]:
            return

        timestamp = self.get_clock().now().to_msg()

        # --- 1. CARTESIAN CONTROL (Sticks) ---
        twist = TwistStamped()
        twist.header.stamp = timestamp
        twist.header.frame_id = self.frame_id
        
        # Left Stick (Axis 1/0) -> Linear X/Y
        twist.twist.linear.x = msg.axes[1] * 0.4
        twist.twist.linear.y = msg.axes[0] * 0.4
        
        # Right Stick (Axis 4/3) -> Linear Z / Angular Yaw
        twist.twist.linear.z = msg.axes[4] * 0.4
        twist.twist.angular.z = msg.axes[3] * 0.4
        
        self.twist_pub.publish(twist)

        # --- 2. JOINT CONTROL (D-Pad) ---
        # D-Pad Horizontal (Axis 6) -> Joint 1 (Base)
        # D-Pad Vertical (Axis 7) -> Joint 2 (Shoulder)
        
        joint_msg = JointJog()
        joint_msg.header.stamp = timestamp
        joint_msg.header.frame_id = self.frame_id
        
        # Only publish joint msg if D-Pad is actually pressed
        if abs(msg.axes[6]) > 0.1 or abs(msg.axes[7]) > 0.1:
            joint_msg.joint_names = ['Joint1', 'Joint2']
            # Map Axis 6 to Joint 1, Axis 7 to Joint 2
            joint_msg.velocities = [msg.axes[6] * 0.5, msg.axes[7] * 0.5]
            self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()