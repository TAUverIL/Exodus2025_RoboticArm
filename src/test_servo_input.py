import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog

class ServoInputTester(Node):
    def __init__(self):
        super().__init__('servo_input_tester')

        # --- CONFIGURATION ---
        # 1. Topic Names (Standard MoveIt Servo defaults)
        # Note: If your servo.yaml sets 'cartesian_command_in_topic' to '/servo_twist',
        # you must change this line to match!
        self.twist_topic = '/servo_node/delta_twist_cmds'
        self.joint_topic = '/servo_node/delta_joint_cmds' 

        # 2. Frame ID (Must match your robot's base frame)
        self.frame_id = 'base_link'
        
        # 3. Joint Names (Must match your URDF)
        self.joint_name = 'Joint1' 

        # --- PUBLISHERS ---
        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.joint_pub = self.create_publisher(JointJog, self.joint_topic, 10)

        # --- TIMER (1 Hz) ---
        self.timer = self.create_timer(1.0, self.publish_commands)
        self.get_logger().info(f'Tester Node Started. Publishing to:\n - {self.twist_topic}\n - {self.joint_topic}')

    def publish_commands(self):
        timestamp = self.get_clock().now().to_msg()

        # 1. Create Twist Command (Cartesian)
        twist_msg = TwistStamped()
        twist_msg.header.stamp = timestamp
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.linear.x = 0.1  # Move slightly in X
        twist_msg.twist.angular.z = 0.1 # Rotate slightly

        # 2. Create Joint Command (Joint Space)
        joint_msg = JointJog()
        joint_msg.header.stamp = timestamp
        joint_msg.header.frame_id = self.frame_id
        joint_msg.joint_names = [self.joint_name]
        joint_msg.velocities = [0.1] # Move Joint1 slightly

        # 3. Publish
        self.twist_pub.publish(twist_msg)
        self.joint_pub.publish(joint_msg)
        
        self.get_logger().info(f'Sent commands at {timestamp.sec}.{timestamp.nanosec}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoInputTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
