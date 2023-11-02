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

#include <panther_hardware_interfaces/gpio_driver.hpp>
#include <panther_hardware_interfaces/panther_wheels_controller.hpp>
#include <panther_hardware_interfaces/canopen_error_filter.hpp>
#include <panther_hardware_interfaces/panther_system_node.hpp>

namespace panther_hardware_interfaces
{
using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

/**
 * @brief Attempts to run operation for max_attempts number of times.
 * operation can throw std::runtime_error, which is caught, and on_error function
 * is executed (for example deinitialization or some other clean up in case of 
 * failure)
 * @returns true if operation was successfully executed, false if it wasn't executed
 * and number of attempts exceeded maximum allowed
 */
bool OperationWithAttempts(
  std::function<void()> operation, unsigned max_attempts, std::function<void()> on_error)
{
  for (unsigned attempts_counter = 0; attempts_counter < max_attempts; ++attempts_counter) {
    try {
      operation();
      return true;
    } catch (std::runtime_error & err) {
      on_error();
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("PantherSystem"), "Operation failed: " << err.what() << ". Attempt "
                                                                  << attempts_counter + 1 << " of "
                                                                  << max_attempts);
    }
  }
  return false;
}

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
  void CheckJointSize();
  void SortJointNames();
  void CheckJointNames();
  void SetInitialValues();
  void CheckInterfaces();
  void ReadDrivetrainSettings();
  void ReadCanOpenSettings();
  void ReadInitializationActivationAttempts();
  void ReadParametersAndCreateCanOpenErrorFilter();

  void UpdateHwStates();
  void UpdateDriverState();
  void UpdateSystemFeedback();

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
  std::shared_ptr<PantherWheelsController> roboteq_controller_;

  DrivetrainSettings drivetrain_settings_;
  CanOpenSettings canopen_settings_;

  std::shared_ptr<CanOpenErrorFilter> canopen_error_filter_;

  PantherSystemNode panther_system_node_;

  // Sometimes SDO errors can happen during initialization and activation of roboteqs, in this cases it is better to retry
  // [ros2_control_node-1] error: SDO abort code 05040000 received on upload request of object 1000 (Device type) to node 02: SDO protocol timed out
  // [ros2_control_node-1] error: SDO abort code 05040000 received on upload request of sub-object 1018:01 (Vendor-ID) to node 02: SDO protocol timed out
  unsigned max_roboteq_initialization_attempts_;
  unsigned max_roboteq_activation_attempts_;

  const unsigned max_safety_stop_attempts_ = 20;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_