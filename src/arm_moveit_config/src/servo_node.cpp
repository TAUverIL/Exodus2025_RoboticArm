#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("servo_node");
  
  // 1. Set up Planning Scene Monitor
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node, "robot_description", tf_buffer, "planning_scene_monitor");
  
  if (psm->getPlanningScene())
  {
    psm->startSceneMonitor();
    psm->startStateMonitor("/joint_states", "/attached_collision_object");
    psm->startWorldGeometryMonitor(); 
    psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning scene not configured");
    return -1;
  }

  // 2. Load Default Parameters (These are Read-Only/Const)
  auto default_params_ptr = moveit_servo::ServoParameters::makeServoParameters(node);
  if (!default_params_ptr)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to load default Servo Parameters");
    return -1;
  }

  // 3. Create a Mutable Copy (So we can edit them)
  moveit_servo::ServoParameters params = *default_params_ptr;

  // --- HARDCODED OVERRIDES ---
  // Now we can modify 'params' because it is our local copy
  params.move_group_name = "arm";
  params.ee_frame_name = "End_effector";           
  params.robot_link_command_frame = "base_link";   
  params.planning_frame = "base_link";
  
  params.command_out_topic = "/arm_controller/joint_trajectory";
  params.command_out_type = "trajectory_msgs/JointTrajectory";
  
  params.cartesian_command_in_topic = "/servo_node/delta_twist_cmds";
  params.joint_command_in_topic = "/servo_node/delta_joint_cmds";
  params.status_topic = "/servo_node/status";

  params.check_collisions = true;
  params.publish_period = 0.04;    
  params.incoming_command_timeout = 0.1; 
  params.linear_scale = 0.4;      
  params.rotational_scale = 0.8;
  // ---------------------------

  // 4. Initialize Servo with our Custom Parameters
  // We must wrap our copy in a shared_ptr to pass it to the constructor
  auto servo = std::make_unique<moveit_servo::Servo>(
      node, 
      std::make_shared<moveit_servo::ServoParameters>(params), 
      psm
  );
  
  servo->start();

  RCLCPP_INFO(node->get_logger(), "------------------------------------------------");
  RCLCPP_INFO(node->get_logger(), "   CUSTOM SERVO NODE STARTED SUCCESSFULLY");
  RCLCPP_INFO(node->get_logger(), "   Move Group: %s", params.move_group_name.c_str());
  RCLCPP_INFO(node->get_logger(), "   EE Frame:   %s", params.ee_frame_name.c_str());
  RCLCPP_INFO(node->get_logger(), "------------------------------------------------");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}