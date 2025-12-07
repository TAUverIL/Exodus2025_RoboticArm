#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped')
        self.subscription = self.create_subscription(
            Twist,
            'servo_twist_unstamped',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(TwistStamped, 'servo_twist', 10)
        self.declare_parameter('frame_id', 'base_link')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

    def listener_callback(self, msg):
        new_msg = TwistStamped()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = self.frame_id
        new_msg.twist = msg
        self.publisher_.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
