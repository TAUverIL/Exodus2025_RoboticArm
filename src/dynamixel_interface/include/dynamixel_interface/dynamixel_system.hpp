#ifndef DYNAMIXEL_INTERFACE__DYNAMIXEL_SYSTEM_HPP_
#define DYNAMIXEL_INTERFACE__DYNAMIXEL_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_interface/visibility_control.h"

namespace dynamixel_interface
{
class DynamixelSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelSystem)

  DYNAMIXEL_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  DYNAMIXEL_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DYNAMIXEL_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DYNAMIXEL_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DYNAMIXEL_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Dynamixel SDK handlers
  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler_;

  // Config parameters
  std::string device_name_;
  int baud_rate_;
  
  // Storage for joint data
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  
  // Mapping from joint index to Dynamixel ID
  std::vector<uint8_t> dxl_ids_;
};

}  // namespace dynamixel_interface

#endif  // DYNAMIXEL_INTERFACE__DYNAMIXEL_SYSTEM_HPP_