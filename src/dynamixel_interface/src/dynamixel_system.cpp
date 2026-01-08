#include "dynamixel_interface/dynamixel_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_interface
{

// Dynamixel Control Table Addresses (Generic X-Series)
// CHECK YOUR MOTOR MANUAL FOR EXACT ADDRESSES!
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

hardware_interface::CallbackReturn DynamixelSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 1. Read parameters from the URDF (hardware_info)
  // Example URDF: <param name="device_name">/dev/ttyUSB0</param>
  device_name_ = info_.hardware_parameters.count("device_name") ?
                 info_.hardware_parameters.at("device_name") : "/dev/ttyUSB0";
  
  baud_rate_ = info_.hardware_parameters.count("baud_rate") ?
               std::stoi(info_.hardware_parameters.at("baud_rate")) : 57600;

  // Initialize storage vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // 2. Read Joint IDs from URDF
  // Example URDF joint param: <param name="id">1</param>
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.parameters.count("id"))
    {
      dxl_ids_.push_back(std::stoi(joint.parameters.at("id")));
    }
    else
    {
      RCLCPP_FATAL(rclcpp::get_logger("DynamixelSystem"), "Joint '%s' missing 'id' parameter", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize PortHandler & PacketHandler
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0); // Protocol 2.0

  // Open Port
  if (!portHandler_->openPort()) {
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "Failed to open port %s", device_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set Baudrate
  if (!portHandler_->setBaudRate(baud_rate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "Failed to set baudrate %d", baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DynamixelSystem"), "Successfully configured Dynamixels!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    
    // Optional: Add velocity if needed
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DynamixelSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  uint8_t dxl_error = 0;

  // --- ADD THIS BLOCK ---
  // Force Operating Mode = 3 (Position Control)
  // Address 11 is "Operating Mode" for XM/XC/XL series
  uint16_t ADDR_OPERATING_MODE = 11;
  uint8_t POSITION_MODE = 4;

  for (auto id : dxl_ids_)
  {
    // 1. Disable Torque first (Mode cannot be changed while Torque is ON)
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);

    // 2. Write Operating Mode
    int result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, POSITION_MODE, &dxl_error);
    
    if (result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "Failed to set Position Mode for ID %d", id);
        return hardware_interface::CallbackReturn::ERROR;
    }
  }
  // --- END ADDITION ---

  // Enable Torque for all motors
  for (auto id : dxl_ids_)
  {
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "Failed to enable torque for ID %d", id);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  
  // Set initial commands... (keep your existing code here)
  for (uint i = 0; i < hw_positions_.size(); i++) {
     if (std::isnan(hw_positions_[i])) {
         hw_commands_[i] = 0.0; 
     } else {
         hw_commands_[i] = hw_positions_[i];
     }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Disable Torque (Optional - depends on safety needs)
  uint8_t dxl_error = 0;
  for (auto id : dxl_ids_)
  {
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Simple Read Loop (SyncRead is more efficient but complex)
  uint8_t dxl_error = 0;
  int32_t dxl_present_position = 0;

  for (uint i = 0; i < dxl_ids_.size(); i++)
  {
    int dxl_comm_result = packetHandler_->read4ByteTxRx(
        portHandler_, dxl_ids_[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);

    if (dxl_comm_result == COMM_SUCCESS) {
      // Convert raw value to radians (assuming 0-4095 maps to 0-2pi or similar)
      // NOTE: You need to adjust this conversion factor based on your motor model!
      // Example for XC330 (4096 resolution): 
      // value * (2 * pi / 4096.0)
      hw_positions_[i] = (dxl_present_position - 2048) * (2.0 * M_PI / 4096.0);    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  for (uint i = 0; i < dxl_ids_.size(); i++)
  {
      if (std::isnan(hw_commands_[i])) continue;

      // 1. Math: Convert Radians to Ticks
      double ticks_per_rad = 4096.0 / (2.0 * M_PI);
      int32_t goal_pos = static_cast<int32_t>((hw_commands_[i] * ticks_per_rad) + 2048);

      // --- DIAGNOSTIC PRINT (The Fix) ---
      // This tells us if ROS is clamping the value before we get it
      RCLCPP_INFO(rclcpp::get_logger("DynamixelSystem"), 
          "ID %d | Request: %.2f rad | Sending: %d ticks", 
          dxl_ids_[i], hw_commands_[i], goal_pos);
      // ----------------------------------

      // Safety Clamp
      // if (goal_pos < 0) goal_pos = 0;
      // if (goal_pos > 4095) goal_pos = 4095;

      // 2. Send Command
      dxl_comm_result = packetHandler_->write4ByteTxRx(
          portHandler_, dxl_ids_[i], ADDR_GOAL_POSITION, (uint32_t)goal_pos, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS) {
          RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "TxRx Fail: %s", packetHandler_->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
          RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "Hardware Error: %s", packetHandler_->getRxPacketError(dxl_error));
      }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace dynamixel_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dynamixel_interface::DynamixelSystem, hardware_interface::SystemInterface)