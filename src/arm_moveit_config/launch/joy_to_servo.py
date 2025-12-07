#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time
import threading

# Enums for Xbox Controller (approximate)
class Axis:
    LEFT_STICK_X = 0
    LEFT_STICK_Y = 1
    LEFT_TRIGGER = 2
    RIGHT_STICK_X = 3
    RIGHT_STICK_Y = 4
    RIGHT_TRIGGER = 5
    D_PAD_X = 6
    D_PAD_Y = 7

class Button:
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    BACK = 6
    START = 7
    POWER = 8
    LEFT_STICK_BTN = 9
    RIGHT_STICK_BTN = 10

class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo')

        # Parameters
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('eef_frame_id', 'end_effector_link') # Adjust as needed
        
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.eef_frame_id = self.get_parameter('eef_frame_id').value
        self.frame_to_publish = self.base_frame_id

        # Joints
        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5']

        # Publishers
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_twist', 10)
        self.joint_pub = self.create_publisher(JointJog, '/servo_joint_jog', 10)
        self.collision_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # Subscriber
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Service Client
        self.servo_start_client = self.create_client(Trigger, '/servo_node/start_servo')
        # self.servo_start_client.wait_for_service(timeout_sec=1.0)
        # self.servo_start_client.call_async(Trigger.Request())

        # State
        self.mode = 'CARTESIAN' # 'CARTESIAN' or 'JOINT'
        self.buttons_prev = [0] * 11

        self.get_logger().info("JoyToServo node started.")
        self.get_logger().info(f"Default Frame: {self.frame_to_publish}")
        self.get_logger().info("Controls:")
        self.get_logger().info("  A: Switch to Cartesian Mode")
        self.get_logger().info("  B: Switch to Joint Mode")
        self.get_logger().info("  Back: Switch to Base Frame")
        self.get_logger().info("  Start: Switch to End Effector Frame")

        # Publish collision scene (optional, similar to C++ example)
        self.publish_collision_scene()

    def publish_collision_scene(self):
        # Simple thread to publish scene after a delay
        def job():
            time.sleep(3)
            # Create collision object (e.g. a floor or table)
            # This is just an example matching the C++ structure
            pass 
        
        thread = threading.Thread(target=job)
        thread.start()

    def update_cmd_frame(self, buttons):
        if buttons[Button.BACK] and self.frame_to_publish == self.eef_frame_id:
            self.frame_to_publish = self.base_frame_id
            self.get_logger().info(f"Switched to Base Frame: {self.base_frame_id}")
        elif buttons[Button.START] and self.frame_to_publish == self.base_frame_id:
            self.frame_to_publish = self.eef_frame_id
            self.get_logger().info(f"Switched to EEF Frame: {self.eef_frame_id}")

    def convert_joy_to_cmd(self, axes, buttons, twist, joint):
        # Mode Switching (A/B)
        if buttons[Button.A] and not self.buttons_prev[Button.A]:
            self.mode = 'CARTESIAN'
            self.get_logger().info("Mode: CARTESIAN")
        elif buttons[Button.B] and not self.buttons_prev[Button.B]:
            self.mode = 'JOINT'
            self.get_logger().info("Mode: JOINT")
        
        self.buttons_prev = list(buttons)
        deadband = 0.2

        if self.mode == 'JOINT':
            # Map sticks to joints
            joint.joint_names = self.joint_names
            velocities = [0.0] * len(self.joint_names)
            
            # Joint 1: Left Stick Hor
            velocities[0] = axes[Axis.LEFT_STICK_X] if abs(axes[Axis.LEFT_STICK_X]) > deadband else 0.0
            # Joint 2: Left Stick Ver
            velocities[1] = axes[Axis.LEFT_STICK_Y] if abs(axes[Axis.LEFT_STICK_Y]) > deadband else 0.0
            # Joint 3: Right Stick Hor
            velocities[2] = axes[Axis.RIGHT_STICK_X] if abs(axes[Axis.RIGHT_STICK_X]) > deadband else 0.0
            # Joint 4: Right Stick Ver
            velocities[3] = axes[Axis.RIGHT_STICK_Y] if abs(axes[Axis.RIGHT_STICK_Y]) > deadband else 0.0
            
            # Joint 5: D-Pad X or Triggers
            # Using Triggers for Joint 5 (LT down, RT up)
            val_j5 = 0.0
            # Triggers are usually -1.0 to 1.0 or 0.0 to 1.0 depending on driver. 
            # Assuming 1.0 is pressed.
            # Standard Joy: Axis 2 (LT) and 5 (RT). 
            # Often initialized at 0.0 or 1.0. Let's assume standard behavior.
            
            # Simple D-Pad fallback if available
            if len(axes) > 6:
                 velocities[4] = axes[Axis.D_PAD_X] if abs(axes[Axis.D_PAD_X]) > deadband else 0.0
            
            joint.velocities = velocities
            return False # Publish Joint

        else: # CARTESIAN
            # Linear
            twist.twist.linear.x = axes[Axis.LEFT_STICK_Y] if abs(axes[Axis.LEFT_STICK_Y]) > deadband else 0.0
            twist.twist.linear.y = axes[Axis.LEFT_STICK_X] if abs(axes[Axis.LEFT_STICK_X]) > deadband else 0.0
            
            # Z axis: Triggers (RT up, LT down)
            # Map -1..1 to 0..1
            rt = -(axes[Axis.RIGHT_TRIGGER] - 1.0) / 2.0 if axes[Axis.RIGHT_TRIGGER] < 0.9 else 0.0 # Assuming 1.0 is released
            lt = -(axes[Axis.LEFT_TRIGGER] - 1.0) / 2.0 if axes[Axis.LEFT_TRIGGER] < 0.9 else 0.0
            
            # Or just use Bumpers for Z like before
            val_z = 0.0
            if buttons[Button.RB]: val_z += 0.5
            if buttons[Button.LB]: val_z -= 0.5
            twist.twist.linear.z = val_z

            # Angular
            twist.twist.angular.y = axes[Axis.RIGHT_STICK_Y] if abs(axes[Axis.RIGHT_STICK_Y]) > deadband else 0.0
            twist.twist.angular.z = axes[Axis.RIGHT_STICK_X] if abs(axes[Axis.RIGHT_STICK_X]) > deadband else 0.0
            
            return True # Publish Twist

    def joy_callback(self, msg):
        twist_msg = TwistStamped()
        joint_msg = JointJog()

        self.update_cmd_frame(msg.buttons)

        if self.convert_joy_to_cmd(msg.axes, msg.buttons, twist_msg, joint_msg):
            # Publish Twist
            twist_msg.header.frame_id = self.frame_to_publish
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(twist_msg)
        else:
            # Publish Joint
            joint_msg.header.frame_id = self.base_frame_id
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
