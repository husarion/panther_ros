// Copyright 2023 Husarion sp. z o.o.
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

#include <panther_hardware_interfaces/panther_system.hpp>

#include <array>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <panther_hardware_interfaces/utils.hpp>

// todo: add user variable to the script that will trigger DOUT4 instead of safety stop

namespace panther_hardware_interfaces
{

void PantherSystem::CheckJointSize() const
{
  if (info_.joints.size() != kJointsSize) {
    throw std::runtime_error(
      "Wrong number of joints defined: " + std::to_string(info_.joints.size()) + ", " +
      std::to_string(kJointsSize) + "expected.");
  }
}

void PantherSystem::SortAndCheckJointNames()
{
  // Sort joints names - later hw_states and hw_commands are accessed by static indexes, so it
  // is necessary to make sure that joints are in specific order and order of definitions in URDF
  // doesn't matter
  for (std::size_t i = 0; i < kJointsSize; i++) {
    std::size_t match_count = 0;

    for (std::size_t j = 0; j < kJointsSize; j++) {
      if (info_.joints[j].name.find(joint_order_[i]) != std::string::npos) {
        joints_names_sorted_[i] = info_.joints[j].name;
        ++match_count;
      }
    }

    if (match_count != 1) {
      throw std::runtime_error(
        "There should be exactly one joint containing " + joint_order_[i] + ", " +
        std::to_string(match_count) + " found.");
    }
  }
}

void PantherSystem::SetInitialValues()
{
  // It isn't safe to set command to NaN - sometimes it could be interpreted as Inf (although it
  // shouldn't). In case of velocity, I think that setting the initial value to 0.0 is the best
  // option.
  hw_commands_velocities_.fill(0.0);

  hw_states_positions_.fill(std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.fill(std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.fill(std::numeric_limits<double>::quiet_NaN());
}

void PantherSystem::CheckInterfaces() const
{
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // Commands
    if (joint.command_interfaces.size() != 1) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + std::to_string(joint.command_interfaces.size()) +
        " command interfaces. 1 expected.");
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + joint.command_interfaces[0].name +
        " command interface. " + hardware_interface::HW_IF_VELOCITY + " expected.");
    }

    // States
    if (joint.state_interfaces.size() != 3) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + std::to_string(joint.state_interfaces.size()) +
        " state  " + (joint.state_interfaces.size() == 1 ? "interface." : "interfaces.") +
        " 3 expected.");
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + joint.state_interfaces[0].name +
        " as first state interface. " + hardware_interface::HW_IF_POSITION + " expected.");
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + joint.state_interfaces[1].name +
        " as second state interface. " + hardware_interface::HW_IF_VELOCITY + " expected.");
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + joint.state_interfaces[2].name +
        " as third state interface. " + hardware_interface::HW_IF_EFFORT + " expected.");
    }
  }
}

void PantherSystem::ReadDrivetrainSettings()
{
  drivetrain_settings_.motor_torque_constant =
    std::stof(info_.hardware_parameters["motor_torque_constant"]);
  drivetrain_settings_.gear_ratio = std::stof(info_.hardware_parameters["gear_ratio"]);
  drivetrain_settings_.gearbox_efficiency =
    std::stof(info_.hardware_parameters["gearbox_efficiency"]);
  drivetrain_settings_.encoder_resolution =
    std::stof(info_.hardware_parameters["encoder_resolution"]);
  drivetrain_settings_.max_rpm_motor_speed =
    std::stof(info_.hardware_parameters["max_rpm_motor_speed"]);
}

void PantherSystem::ReadCANopenSettings()
{
  canopen_settings_.can_interface_name = info_.hardware_parameters["can_interface_name"];
  canopen_settings_.master_can_id = std::stoi(info_.hardware_parameters["master_can_id"]);
  canopen_settings_.front_driver_can_id =
    std::stoi(info_.hardware_parameters["front_driver_can_id"]);
  canopen_settings_.rear_driver_can_id = std::stoi(info_.hardware_parameters["rear_driver_can_id"]);
  canopen_settings_.pdo_feedback_timeout_ms =
    std::chrono::milliseconds(std::stoi(info_.hardware_parameters["pdo_feedback_timeout_ms"]));
  canopen_settings_.sdo_operation_timeout_ms =
    std::chrono::milliseconds(std::stoi(info_.hardware_parameters["sdo_operation_timeout_ms"]));
}

void PantherSystem::ReadInitializationActivationAttempts()
{
  max_roboteq_initialization_attempts_ =
    std::stoi(info_.hardware_parameters["max_roboteq_initialization_attempts"]);
  max_roboteq_activation_attempts_ =
    std::stoi(info_.hardware_parameters["max_roboteq_activation_attempts"]);
  max_safety_stop_attempts_ = std::stoi(info_.hardware_parameters["max_safety_stop_attempts"]);
}

void PantherSystem::ReadParametersAndCreateRoboteqErrorFilter()
{
  const unsigned max_write_sdo_errors_count =
    std::stoi(info_.hardware_parameters["max_write_sdo_errors_count"]);
  const unsigned max_read_sdo_errors_count =
    std::stoi(info_.hardware_parameters["max_read_sdo_errors_count"]);
  const unsigned max_read_pdo_errors_count =
    std::stoi(info_.hardware_parameters["max_read_pdo_errors_count"]);

  roboteq_error_filter_ = std::make_shared<RoboteqErrorFilter>(
    max_read_sdo_errors_count, max_write_sdo_errors_count, max_read_pdo_errors_count, 1);
}

CallbackReturn PantherSystem::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_INFO(logger_, "Initializing");

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    CheckJointSize();
    SortAndCheckJointNames();
    SetInitialValues();
    CheckInterfaces();
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(logger_, "Exception during initialization: " << e.what());
    return CallbackReturn::ERROR;
  }

  try {
    ReadDrivetrainSettings();
    ReadCANopenSettings();
    ReadInitializationActivationAttempts();
    ReadParametersAndCreateRoboteqErrorFilter();
  } catch (const std::invalid_argument & e) {
    RCLCPP_FATAL(logger_, "One of the required hardware parameters was not defined");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");

  motors_controller_ = std::make_shared<MotorsController>(canopen_settings_, drivetrain_settings_);

  panther_system_ros_interface_.Initialize();

  RCLCPP_INFO(logger_, "Initializing Roboteq drivers");

  if (!OperationWithAttempts(
        std::bind(&MotorsController::Initialize, motors_controller_),
        max_roboteq_initialization_attempts_,
        std::bind(&MotorsController::Deinitialize, motors_controller_))) {
    RCLCPP_FATAL_STREAM(logger_, "Initialization failed");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");

  motors_controller_->Deinitialize();
  motors_controller_.reset();

  panther_system_ros_interface_.Deinitialize();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");

  hw_commands_velocities_.fill(0.0);
  hw_states_positions_.fill(0.0);
  hw_states_velocities_.fill(0.0);
  hw_states_efforts_.fill(0.0);

  RCLCPP_INFO(logger_, "Activating Roboteq drivers");

  if (!OperationWithAttempts(
        std::bind(&MotorsController::Activate, motors_controller_),
        max_roboteq_activation_attempts_)) {
    RCLCPP_FATAL_STREAM(logger_, "Activation failed");
    return CallbackReturn::FAILURE;
  }

  panther_system_ros_interface_.Activate(
    std::bind(&RoboteqErrorFilter::SetClearErrorsFlag, roboteq_error_filter_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");

  try {
    motors_controller_->TurnOnSafetyStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Deactivation failed: " << e.what());
    return CallbackReturn::FAILURE;
  }

  panther_system_ros_interface_.Deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  try {
    motors_controller_->TurnOnSafetyStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Shutdown failed: " << e.what());
    return CallbackReturn::FAILURE;
  }

  panther_system_ros_interface_.Deactivate();

  motors_controller_->Deinitialize();
  motors_controller_.reset();

  panther_system_ros_interface_.Deinitialize();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Handling error");

  RCLCPP_INFO(logger_, "Setting safe stop");
  if (!OperationWithAttempts(
        std::bind(&MotorsController::TurnOnSafetyStop, motors_controller_),
        max_safety_stop_attempts_)) {
    RCLCPP_FATAL_STREAM(logger_, "Setting safety stop failed");
    return CallbackReturn::FAILURE;
  }

  panther_system_ros_interface_.Deactivate();

  motors_controller_->Deinitialize();
  motors_controller_.reset();

  panther_system_ros_interface_.Deinitialize();

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> PantherSystem::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (std::size_t i = 0; i < kJointsSize; i++) {
    state_interfaces.emplace_back(StateInterface(
      joints_names_sorted_[i], hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(StateInterface(
      joints_names_sorted_[i], hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(StateInterface(
      joints_names_sorted_[i], hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<CommandInterface> PantherSystem::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < kJointsSize; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joints_names_sorted_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

void PantherSystem::UpdateDriverState()
{
  try {
    // Other feedback values are read through SDO - it requires more time, so instead of
    // reading all of them at once, every read cycle one value is updated, once all of them
    // are read, feedback message is updated. As there are 8 values, the frequency of updates
    // will be controller_frequency / 8
    const bool finished_updates = motors_controller_->UpdateDriversState();

    if (finished_updates) {
      panther_system_ros_interface_.UpdateMsgDriversParameters(
        motors_controller_->GetFrontData().GetDriverState(),
        motors_controller_->GetRearData().GetDriverState());
    }

    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_SDO, false);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Error when trying to read drivers feedback: " << e.what());
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_SDO, true);
  }
}

void PantherSystem::UpdateSystemFeedback()
{
  try {
    motors_controller_->UpdateSystemFeedback();
    UpdateHwStates();
    panther_system_ros_interface_.UpdateMsgErrorFlags(
      motors_controller_->GetFrontData(), motors_controller_->GetRearData());

    if (
      motors_controller_->GetFrontData().IsFlagError() ||
      motors_controller_->GetRearData().IsFlagError()) {
      RCLCPP_ERROR_STREAM_THROTTLE(
        logger_, steady_clock_, 5000,
        "Error state on one of the drivers"
          << "\nFront: " << motors_controller_->GetFrontData().GetFlagErrorLog()
          << "\nRear: " << motors_controller_->GetRearData().GetFlagErrorLog());
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, true);
    } else {
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, false);
    }

    if (
      motors_controller_->GetFrontData().IsDataTimedOut() ||
      motors_controller_->GetRearData().IsDataTimedOut()) {
      RCLCPP_ERROR_STREAM(
        logger_, "PDO data timeout: "
                   << (motors_controller_->GetFrontData().IsDataTimedOut() ? "front " : "")
                   << (motors_controller_->GetRearData().IsDataTimedOut() ? "rear" : ""));
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO, true);
    } else {
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO, false);
    }
  } catch (const std::runtime_error & e) {
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO, true);
    RCLCPP_ERROR_STREAM(logger_, "Error when trying to read feedback: " << e.what());
  }
}

void PantherSystem::UpdateMsgErrors()
{
  CANErrors can_errors;
  can_errors.error = roboteq_error_filter_->IsError();
  can_errors.write_sdo_error = roboteq_error_filter_->IsError(ErrorsFilterIds::WRITE_SDO);
  can_errors.read_sdo_error = roboteq_error_filter_->IsError(ErrorsFilterIds::READ_SDO);
  can_errors.read_pdo_error = roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO);
  can_errors.front_data_timed_out = motors_controller_->GetFrontData().IsDataTimedOut();
  can_errors.rear_data_timed_out = motors_controller_->GetRearData().IsDataTimedOut();
  can_errors.front_can_net_err = motors_controller_->GetFrontData().IsCANNetErr();
  can_errors.rear_can_net_err = motors_controller_->GetRearData().IsCANNetErr();

  panther_system_ros_interface_.UpdateMsgErrors(can_errors);
}

return_type PantherSystem::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  UpdateDriverState();
  UpdateSystemFeedback();
  UpdateMsgErrors();

  panther_system_ros_interface_.PublishDriverState();

  return return_type::OK;
}

return_type PantherSystem::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (!roboteq_error_filter_->IsError()) {
    try {
      motors_controller_->WriteSpeed(
        hw_commands_velocities_[0], hw_commands_velocities_[1], hw_commands_velocities_[2],
        hw_commands_velocities_[3]);
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::WRITE_SDO, false);
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(logger_, "Error when trying to write commands: " << e.what());
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::WRITE_SDO, true);
    }
  }

  // "soft" error - still there is communication over CAN with drivers, so publishing feedback is
  // continued - hardware interface's onError isn't triggered estop is handled similarly - at the
  // time of writing there wasn't a better approach to handling estop. Moved after WriteSpeed, so
  // that safety stop can be set just after write error happens.
  if (roboteq_error_filter_->IsError()) {
    if (!CheckIfSafetyStopActive()) {
      RCLCPP_ERROR(logger_, "Sending safety stop request");
      // 0 command is set with safety stop
      try {
        motors_controller_->TurnOnSafetyStop();
      } catch (const std::runtime_error & e) {
        RCLCPP_FATAL_STREAM(logger_, "Error when trying to turn on safety stop: " << e.what());
        return return_type::ERROR;
      }
    }

    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, steady_clock_, 5000, "Error detected, ignoring write commands");
    return return_type::OK;
  }

  return return_type::OK;
}

bool PantherSystem::CheckIfSafetyStopActive()
{
  return motors_controller_->GetFrontData().GetLeftRuntimeError().GetMessage().safety_stop_active &&
         motors_controller_->GetFrontData()
           .GetRightRuntimeError()
           .GetMessage()
           .safety_stop_active &&
         motors_controller_->GetRearData().GetLeftRuntimeError().GetMessage().safety_stop_active &&
         motors_controller_->GetRearData().GetRightRuntimeError().GetMessage().safety_stop_active;
}

void PantherSystem::UpdateHwStates()
{
  const auto front = motors_controller_->GetFrontData();
  const auto rear = motors_controller_->GetRearData();

  const auto fl = front.GetLeftMotorState();
  const auto fr = front.GetRightMotorState();
  const auto rl = rear.GetLeftMotorState();
  const auto rr = rear.GetRightMotorState();

  hw_states_positions_[0] = fl.GetPosition();
  hw_states_positions_[1] = fr.GetPosition();
  hw_states_positions_[2] = rl.GetPosition();
  hw_states_positions_[3] = rr.GetPosition();

  hw_states_velocities_[0] = fl.GetVelocity();
  hw_states_velocities_[1] = fr.GetVelocity();
  hw_states_velocities_[2] = rl.GetVelocity();
  hw_states_velocities_[3] = rr.GetVelocity();

  hw_states_efforts_[0] = fl.GetTorque();
  hw_states_efforts_[1] = fr.GetTorque();
  hw_states_efforts_[2] = rl.GetTorque();
  hw_states_efforts_[3] = rr.GetTorque();
}

}  // namespace panther_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  panther_hardware_interfaces::PantherSystem, hardware_interface::SystemInterface)
