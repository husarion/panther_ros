// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_HPP_
#define PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_HPP_

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "panther_hardware_interfaces/gpio_controller.hpp"
#include "panther_hardware_interfaces/motors_controller.hpp"
#include "panther_hardware_interfaces/panther_system_e_stop.hpp"
#include "panther_hardware_interfaces/panther_system_ros_interface.hpp"
#include "panther_hardware_interfaces/roboteq_error_filter.hpp"

namespace panther_hardware_interfaces
{

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

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

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & /* period */) override;
  return_type write(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) override;

protected:
  void CheckJointSize() const;
  void SortAndCheckJointNames();
  void SetInitialValues();
  void CheckInterfaces() const;
  void ReadPantherVersion();
  void ReadDrivetrainSettings();
  void ReadCANopenSettings();
  void ReadInitializationActivationAttempts();
  void ReadParametersAndCreateRoboteqErrorFilter();
  void ReadDriverStatesUpdateFrequency();

  void ConfigureGPIOController();
  void ConfigureMotorsController();
  void ConfigureEStop();

  void UpdateMotorsStates();
  void UpdateDriverState();

  void UpdateHwStates();
  void UpdateMotorsStatesDataTimedOut();
  bool AreVelocityCommandsNearZero();
  bool IsPantherVersionAtLeast(const float version);

  void UpdateDriverStateMsg();
  void UpdateFlagErrors();
  void UpdateDriverStateDataTimedOut();

  void HandlePDOWriteOperation(std::function<void()> pdo_write_operation);

  void MotorsPowerEnable(const bool enable);

  void DiagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status);
  void DiagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status);

  static constexpr size_t kJointsSize = 4;

  // Currently only velocity command mode is supported - although Roboteq driver support position
  // and torque mode, in 2.1 firmware both modes aren't really stable and safe.
  // In torque mode sometimes after killing the software motor moves and it generally isn't well
  // tuned. Position mode also isn't really stable (reacts abruptly to spikes). If updating the
  // firmware to 2.1a will solve these issues, it may be worth to enable other modes.
  std::array<double, kJointsSize> hw_commands_velocities_;

  std::array<double, kJointsSize> hw_states_positions_;
  std::array<double, kJointsSize> hw_states_velocities_;
  std::array<double, kJointsSize> hw_states_efforts_;

  // Define expected joint order, so that it doesn't matter what order is defined in the URDF.
  // It is expected that the joint name should contain these specifiers.
  static const inline std::array<std::string, kJointsSize> joint_order_ = {"fl", "fr", "rl", "rr"};
  std::array<std::string, kJointsSize> joints_names_sorted_;

  std::shared_ptr<GPIOControllerInterface> gpio_controller_;
  std::shared_ptr<MotorsController> motors_controller_;
  std::shared_ptr<EStopInterface> e_stop_;

  DrivetrainSettings drivetrain_settings_;
  CANopenSettings canopen_settings_;

  std::unique_ptr<PantherSystemRosInterface> panther_system_ros_interface_;

  // Sometimes SDO errors can happen during initialization and activation of Roboteq drivers,
  // in these cases it is better to retry
  // Example errors:
  // SDO abort code 05040000 received on upload request of object 1000 (Device type) to
  // node 02: SDO protocol timed out
  // SDO abort code 05040000 received on upload request of sub-object 1018:01 (Vendor-ID) to
  // node 02: SDO protocol timed out
  unsigned max_roboteq_initialization_attempts_;
  unsigned max_roboteq_activation_attempts_;

  rclcpp::Logger logger_{rclcpp::get_logger("PantherSystem")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::shared_ptr<RoboteqErrorFilter> roboteq_error_filter_;

  float panther_version_;

  std::shared_ptr<std::mutex> motor_controller_write_mtx_;

  rclcpp::Time next_driver_state_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration driver_states_update_period_{0, 0};
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_HPP_
