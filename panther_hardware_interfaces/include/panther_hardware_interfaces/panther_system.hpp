#ifndef PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_
#define PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_

#include <panther_hardware_interfaces/visibility_control.hpp>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <panther_hardware_interfaces/gpio_driver.hpp>
#include <panther_hardware_interfaces/panther_wheels_controller.hpp>

namespace panther_hardware_interfaces
{
using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

class PantherSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PantherSystem)

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  std::vector<StateInterface> export_state_interfaces() override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  std::vector<CommandInterface> export_command_interfaces() override;

  // Mode switch currently disabled - torque and position mode in Roboteq isn't working well
  // enough, so only velocity mode will be supported. New firmware (2.1a) may fix some bugs,
  // then we will reconsider enabling it

  // PANTHER_HARDWARE_INTERFACES_PUBLIC
  // return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;

  // PANTHER_HARDWARE_INTERFACES_PUBLIC
  // return_type perform_command_mode_switch(
  //   const std::vector<std::string> &, const std::vector<std::string> &) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // As described above prepare_command_mode_switch
  // std::map<std::string, double> pos_commands_;
  // std::map<std::string, double> effort_commands_;

  // TODO: check if RT safe
  std::map<std::string, double> vel_commands_;

  std::map<std::string, double> pos_state_;
  std::map<std::string, double> vel_state_;
  std::map<std::string, double> effort_state_;

  std::unique_ptr<GPIOController> gpio_controller_;
  std::unique_ptr<PantherWheelsController> roboteq_controller_;

  std::string hardware_interface_type_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_