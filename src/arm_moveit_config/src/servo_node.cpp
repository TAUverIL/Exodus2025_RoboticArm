#include <memory>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node_options =
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("servo_node", node_options);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          node, "robot_description", tf_buffer, "planning_scene_monitor");

  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR(node->get_logger(), "Planning scene not configured");
    rclcpp::shutdown();
    return 1;
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor(
      "/joint_states", "/attached_collision_object");

  auto default_params = moveit_servo::ServoParameters::makeServoParameters(node);
  if (!default_params)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to load Servo parameters");
    rclcpp::shutdown();
    return 1;
  }

  moveit_servo::ServoParameters params = *default_params;

  params.move_group_name = "arm";
  params.planning_frame = "base_link";
  params.ee_frame_name = "Link_5";
  params.robot_link_command_frame = "base_link";

  params.command_out_topic = "/servo_velocity_controller/commands";
  params.command_out_type = "std_msgs/Float64MultiArray";
  params.publish_joint_positions = false;
  params.publish_joint_velocities = true;
  params.publish_joint_accelerations = false;

  params.cartesian_command_in_topic = "/servo_node/delta_twist_cmds";
  params.joint_command_in_topic = "/servo_node/delta_joint_cmds";
  params.status_topic = "/servo_node/status";
  params.joint_topic = "/joint_states";

  params.command_in_type = "unitless";
  params.linear_scale = 0.4;
  params.rotational_scale = 0.8;
  params.joint_scale = 0.5;
  params.publish_period = 0.02;
  params.low_latency_mode = true;
  params.incoming_command_timeout = 0.1;
  params.num_outgoing_halt_msgs_to_publish = 4;

  params.check_collisions = false;
  params.collision_check_rate = 10.0;
  params.self_collision_proximity_threshold = 0.01;
  params.scene_collision_proximity_threshold = 0.02;
  params.is_primary_planning_scene_monitor = false;

  auto servo = std::make_unique<moveit_servo::Servo>(
      node, std::make_shared<moveit_servo::ServoParameters>(params),
      planning_scene_monitor);
  servo->start();

  RCLCPP_INFO(node->get_logger(), "Custom hard-coded Servo node started");
  RCLCPP_INFO(node->get_logger(), "Move group: %s", params.move_group_name.c_str());
  RCLCPP_INFO(node->get_logger(), "EE frame: %s", params.ee_frame_name.c_str());
  RCLCPP_INFO(node->get_logger(), "Twist input: %s",
              params.cartesian_command_in_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "Joint input: %s",
              params.joint_command_in_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "Command output: %s",
              params.command_out_topic.c_str());

  rclcpp::spin(node);
  servo.reset();
  planning_scene_monitor.reset();
  rclcpp::shutdown();
  return 0;
}
