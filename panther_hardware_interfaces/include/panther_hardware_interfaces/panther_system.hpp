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

// TODO: move it somewhere
/**
 * @brief Attempts to run operation for max_attempts number of times.
 * operation can throw std::runtime_error, which is caught, and on_error function
 * is executed (for example deinitialization or some other clean up in case of 
 * failure)
 * @return true if operation was successfully executed, false if it wasn't executed
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

/**
 * @brief Class that implements SystemInterface from ros2_control for Panther
 */
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

  // Currently only velocity command mode is supported - although roboteq driver support position
  // and torque mode, in 2.1 firmware both modes aren't really stable and safe.
  // In torque mode sometimes after killing software motor moves and it generally isn't well tuned.
  // Position mode also isn't really stable (reacts abruptly to spikes).
  // If updating firmware to 2.1a will solve this issues, it may be worth to support other modes.
  double hw_commands_velocities_[kJointsSize];

  double hw_states_positions_[kJointsSize];
  double hw_states_velocities_[kJointsSize];
  double hw_states_efforts_[kJointsSize];

  // Define expected joint order, so that it doesn't mattter order defined in the URDF
  // it is expected that joint name should contain these specifiers
  std::string joint_order_[kJointsSize] = {"fl", "fr", "rl", "rr"};
  std::string joints_names_sorted_[kJointsSize];

  std::unique_ptr<GPIOController> gpio_controller_;
  std::shared_ptr<PantherWheelsController> roboteq_controller_;

  DrivetrainSettings drivetrain_settings_;
  CanOpenSettings canopen_settings_;

  std::shared_ptr<CanOpenErrorFilter> canopen_error_filter_;

  PantherSystemNode panther_system_node_;

  // Sometimes SDO errors can happen during initialization and activation of roboteq drivers,
  // in this cases it is better to retry
  // Example errors:
  // SDO abort code 05040000 received on upload request of object 1000 (Device type) to
  // node 02: SDO protocol timed out
  // SDO abort code 05040000 received on upload request of sub-object 1018:01 (Vendor-ID) to
  // node 02: SDO protocol timed out
  unsigned max_roboteq_initialization_attempts_ = 2;
  unsigned max_roboteq_activation_attempts_ = 2;

  // SDO error can happen also during setting safetry stop (it may be not necessary to do attempts
  // once we have GPIO controller)
  unsigned max_safety_stop_attempts_ = 20;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_