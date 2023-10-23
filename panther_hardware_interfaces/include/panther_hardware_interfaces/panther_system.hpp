#ifndef PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_
#define PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <realtime_tools/realtime_publisher.h>

#include <panther_msgs/msg/driver_state.hpp>

#include <panther_hardware_interfaces/gpio_driver.hpp>
#include <panther_hardware_interfaces/panther_wheels_controller.hpp>
#include <panther_hardware_interfaces/panther_system_error_handler.hpp>

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

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<StateInterface> export_state_interfaces() override;
  std::vector<CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void UpdateHwStates();
  void UpdateMsgDriversErrorsState();
  void UpdateMsgDriversState();
  void UpdateMsgErrors();
  void PublishDriverState();

  static constexpr size_t kJointsSize = 4;

  // consider adding position and torque mode after updating roboteq firmware to 2.1a
  // In 2.1 both position and torque mode aren't really stable and safe
  // in torque mode sometimes after killing software motor moves and it generally isn't well tuned
  // position mode also isn't really stable (reacts abruptly to spikes, which we hope will be fixed
  // in the new firmware)

  double hw_commands_velocities_[kJointsSize];

  double hw_states_positions_[kJointsSize];
  double hw_states_velocities_[kJointsSize];
  double hw_states_efforts_[kJointsSize];

  // Define expected joint order, so that it doesn't mattter order defined in the panther_macro
  // it is expected that joint name should contain these specifiers
  std::string joint_order_[kJointsSize] = {"fl", "fr", "rl", "rr"};
  std::string joints_names_sorted_[kJointsSize];

  std::unique_ptr<GPIOController> gpio_controller_;
  std::unique_ptr<PantherWheelsController> roboteq_controller_;

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<std::thread> executor_thread_;

  std::shared_ptr<rclcpp::Publisher<panther_msgs::msg::DriverState>> driver_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<panther_msgs::msg::DriverState>>
    realtime_driver_state_publisher_;

  DrivetrainSettings drivetrain_settings_;
  CanSettings can_settings_;

  void ResetPublishers();
  void DestroyNode();
  std::atomic_bool stop_executor_ = false;

  std::unique_ptr<PantherSystemErrorHandler> error_handler_;

  // TODO test

  // Sometimes SDO errors can happen during initialization and activation of roboteqs, in this cases it is better to retry
  // [ros2_control_node-1] error: SDO abort code 05040000 received on upload request of object 1000 (Device type) to node 02: SDO protocol timed out
  // [ros2_control_node-1] error: SDO abort code 05040000 received on upload request of sub-object 1018:01 (Vendor-ID) to node 02: SDO protocol timed out
  unsigned max_roboteq_initialization_attempts_;
  unsigned max_roboteq_activation_attempts_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_