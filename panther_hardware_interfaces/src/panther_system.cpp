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
      if (CheckIfJointNameContainValidSequence(info_.joints[j].name, joint_order_[i])) {
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

void PantherSystem::ReadPantherVersion()
{
  panther_version_ = std::stof(info_.hardware_parameters["panther_version"]);
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
  canopen_settings_.pdo_motor_states_timeout_ms =
    std::chrono::milliseconds(std::stoi(info_.hardware_parameters["pdo_motor_states_timeout_ms"]));
  canopen_settings_.pdo_driver_state_timeout_ms =
    std::chrono::milliseconds(std::stoi(info_.hardware_parameters["pdo_driver_state_timeout_ms"]));
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
  const unsigned max_write_pdo_cmds_errors_count =
    std::stoi(info_.hardware_parameters["max_write_pdo_cmds_errors_count"]);
  const unsigned max_read_pdo_motor_states_errors_count =
    std::stoi(info_.hardware_parameters["max_read_pdo_motor_states_errors_count"]);
  const unsigned max_read_pdo_driver_state_errors_count =
    std::stoi(info_.hardware_parameters["max_read_pdo_driver_state_errors_count"]);

  roboteq_error_filter_ = std::make_shared<RoboteqErrorFilter>(
    max_write_pdo_cmds_errors_count, max_read_pdo_motor_states_errors_count,
    max_read_pdo_driver_state_errors_count, 1);
}

CallbackReturn PantherSystem::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_INFO(logger_, "Initializing Panther System");

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
    ReadPantherVersion();
    ReadDrivetrainSettings();
    ReadCANopenSettings();
    ReadInitializationActivationAttempts();
    ReadParametersAndCreateRoboteqErrorFilter();

    const float driver_states_update_frequency =
      std::stof(info_.hardware_parameters["driver_states_update_frequency"]);
    driver_states_update_period_ =
      rclcpp::Duration::from_seconds(1.0f / driver_states_update_frequency);

  } catch (const std::invalid_argument & e) {
    RCLCPP_FATAL(logger_, "One of the required hardware parameters was not defined");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring Panther System");

  RCLCPP_INFO_STREAM(logger_, "Creating GPIO controller for Panther version: " << panther_version_);
  if (panther_version_ >= 1.2 - std::numeric_limits<float>::epsilon()) {
    gpio_controller_ = std::make_shared<GPIOControllerPTH12X>();
  } else {
    gpio_controller_ = std::make_shared<GPIOControllerPTH10X>();
  }

  gpio_controller_->Start();

  motors_controller_ = std::make_shared<MotorsController>(canopen_settings_, drivetrain_settings_);

  if (!OperationWithAttempts(
        std::bind(&MotorsController::Initialize, motors_controller_),
        max_roboteq_initialization_attempts_,
        std::bind(&MotorsController::Deinitialize, motors_controller_))) {
    RCLCPP_FATAL_STREAM(logger_, "Roboteq drivers initialization failed");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up Panther System");

  motors_controller_->Deinitialize();
  motors_controller_.reset();

  gpio_controller_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating Panther System");

  hw_commands_velocities_.fill(0.0);
  hw_states_positions_.fill(0.0);
  hw_states_velocities_.fill(0.0);
  hw_states_efforts_.fill(0.0);

  RCLCPP_INFO(logger_, "Activating Roboteq drivers");

  if (!OperationWithAttempts(
        std::bind(&MotorsController::Activate, motors_controller_),
        max_roboteq_activation_attempts_)) {
    RCLCPP_FATAL_STREAM(logger_, "Activation failed");
    return CallbackReturn::ERROR;
  }

  panther_system_ros_interface_ =
    std::make_shared<PantherSystemRosInterface>("panther_system_node");

  gpio_controller_->ConfigureGpioStateCallback(std::bind(
    &PantherSystemRosInterface::PublishGPIOState, panther_system_ros_interface_,
    std::placeholders::_1));

  panther_system_ros_interface_->AddSetBoolService(
    "~/motor_power_enable",
    std::bind(&GPIOControllerInterface::MotorsEnable, gpio_controller_, std::placeholders::_1));
  panther_system_ros_interface_->AddSetBoolService(
    "~/fan_enable",
    std::bind(&GPIOControllerInterface::FanEnable, gpio_controller_, std::placeholders::_1));
  panther_system_ros_interface_->AddSetBoolService(
    "~/aux_power_enable",
    std::bind(&GPIOControllerInterface::AUXEnable, gpio_controller_, std::placeholders::_1));
  panther_system_ros_interface_->AddSetBoolService(
    "~/digital_power_enable",
    std::bind(&GPIOControllerInterface::VDIGEnable, gpio_controller_, std::placeholders::_1));
  panther_system_ros_interface_->AddSetBoolService(
    "~/charger_enable",
    std::bind(&GPIOControllerInterface::ChargerEnable, gpio_controller_, std::placeholders::_1));

  panther_system_ros_interface_->AddTriggerService(
    "~/e_stop_trigger", std::bind(&PantherSystem::SetEStop, this));
  panther_system_ros_interface_->AddTriggerService(
    "~/e_stop_reset", std::bind(&PantherSystem::ResetEStop, this));

  panther_system_ros_interface_->InitializeAndPublishIOStateMsg(gpio_controller_, panther_version_);

  estop_ = ReadEStop();
  panther_system_ros_interface_->InitializeAndPublishEstopStateMsg(estop_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating Panther System");

  try {
    SetEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Deactivation failed: " << e.what());
    return CallbackReturn::ERROR;
  }

  panther_system_ros_interface_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down Panther System");
  try {
    SetEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Shutdown failed: " << e.what());
    return CallbackReturn::ERROR;
  }

  panther_system_ros_interface_.reset();

  motors_controller_->Deinitialize();
  motors_controller_.reset();

  gpio_controller_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Handling Panther System error");

  if (!OperationWithAttempts(
        std::bind(&PantherSystem::SetEStop, this), max_safety_stop_attempts_)) {
    RCLCPP_FATAL_STREAM(logger_, "Setting safety stop failed");
    return CallbackReturn::ERROR;
  }

  panther_system_ros_interface_.reset();

  motors_controller_->Deinitialize();
  motors_controller_.reset();

  gpio_controller_.reset();

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

void PantherSystem::UpdateMotorsStates()
{
  try {
    motors_controller_->UpdateMotorsStates();
    UpdateHwStates();

    if (
      motors_controller_->GetFrontData().IsMotorStatesDataTimedOut() ||
      motors_controller_->GetRearData().IsMotorStatesDataTimedOut()) {
      RCLCPP_WARN_STREAM(
        logger_, (motors_controller_->GetFrontData().IsMotorStatesDataTimedOut() ? "Front " : "")
                   << (motors_controller_->GetRearData().IsMotorStatesDataTimedOut() ? "Rear " : "")
                   << "PDO motor state data timeout");
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);
    } else {
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, false);
    }

  } catch (const std::runtime_error & e) {
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);
    RCLCPP_WARN_STREAM(logger_, "Error when trying to read motors states: " << e.what());
  }
}

void PantherSystem::UpdateDriverState()
{
  try {
    motors_controller_->UpdateDriversState();

    panther_system_ros_interface_->UpdateMsgDriversStates(
      motors_controller_->GetFrontData().GetDriverState(),
      motors_controller_->GetRearData().GetDriverState());

    panther_system_ros_interface_->UpdateMsgErrorFlags(
      motors_controller_->GetFrontData(), motors_controller_->GetRearData());

    CANErrors can_errors;
    can_errors.error = roboteq_error_filter_->IsError();
    can_errors.write_pdo_cmds_error =
      roboteq_error_filter_->IsError(ErrorsFilterIds::WRITE_PDO_CMDS);
    can_errors.read_pdo_motor_states_error =
      roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES);
    can_errors.read_pdo_driver_state_error =
      roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE);

    can_errors.front_motor_states_data_timed_out =
      motors_controller_->GetFrontData().IsMotorStatesDataTimedOut();
    can_errors.rear_motor_states_data_timed_out =
      motors_controller_->GetRearData().IsMotorStatesDataTimedOut();

    can_errors.front_driver_state_data_timed_out =
      motors_controller_->GetFrontData().IsDriverStateDataTimedOut();
    can_errors.rear_driver_state_data_timed_out =
      motors_controller_->GetRearData().IsDriverStateDataTimedOut();

    can_errors.front_can_net_err = motors_controller_->GetFrontData().IsCANNetErr();
    can_errors.rear_can_net_err = motors_controller_->GetRearData().IsCANNetErr();

    panther_system_ros_interface_->UpdateMsgErrors(can_errors);

    if (
      motors_controller_->GetFrontData().IsFlagError() ||
      motors_controller_->GetRearData().IsFlagError()) {
      RCLCPP_WARN_STREAM_THROTTLE(
        logger_, steady_clock_, 5000,
        "Error state on one of the drivers:\n"
          << "\tFront: " << motors_controller_->GetFrontData().GetFlagErrorLog()
          << "\tRear: " << motors_controller_->GetRearData().GetFlagErrorLog());
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, true);
    } else {
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, false);
    }

    if (
      motors_controller_->GetFrontData().IsDriverStateDataTimedOut() ||
      motors_controller_->GetRearData().IsDriverStateDataTimedOut()) {
      RCLCPP_WARN_STREAM(
        logger_, (motors_controller_->GetFrontData().IsDriverStateDataTimedOut() ? "Front " : "")
                   << (motors_controller_->GetRearData().IsDriverStateDataTimedOut() ? "Rear " : "")
                   << "PDO driver state timeout");
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, true);
    } else {
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, false);
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN_STREAM(logger_, "Error when trying to read drivers states: " << e.what());
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, true);
  }

  estop_ = ReadEStop();
  if (estop_) {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, steady_clock_, 5000, "EStop active");
  }

  panther_system_ros_interface_->PublishDriverState();
  panther_system_ros_interface_->PublishEstopStateIfChanged(estop_);
}

return_type PantherSystem::read(const rclcpp::Time & time, const rclcpp::Duration & /* period */)
{
  UpdateMotorsStates();

  if (time >= next_driver_state_update_time_) {
    UpdateDriverState();
    next_driver_state_update_time_ = time + driver_states_update_period_;
  }

  return return_type::OK;
}

return_type PantherSystem::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (!roboteq_error_filter_->IsError()) {
    try {
      motors_controller_->SendSpeedCommands(
        hw_commands_velocities_[0], hw_commands_velocities_[1], hw_commands_velocities_[2],
        hw_commands_velocities_[3]);
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, false);
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN_STREAM(logger_, "Error when trying to write commands: " << e.what());
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);
    }
  }

  // "soft" error - still there is communication over CAN with drivers, so publishing feedback is
  // continued - hardware interface's onError isn't triggered estop is handled similarly - at the
  // time of writing there wasn't a better approach to handling estop. Moved after
  // SendSpeedCommands, so that safety stop can be set just after write error happens.
  if (roboteq_error_filter_->IsError()) {
    if (!CheckIfSafetyStopActive()) {
      RCLCPP_ERROR(
        logger_,
        "Error detected and at least on of the channels is not in the safety stop state, sending "
        "safety stop request...");
      // 0 command is set with safety stop
      try {
        SetEStop();
      } catch (const std::runtime_error & e) {
        RCLCPP_FATAL_STREAM(logger_, "Error when trying to turn on safety stop: " << e.what());
        return return_type::ERROR;
      }
    }

    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, steady_clock_, 5000, "Error detected, ignoring write commands");
    return return_type::OK;
  }

  if (!estop_) {
    try {
      {
        std::unique_lock<std::mutex> motor_controller_write_lck(
          motor_controller_write_mtx_, std::defer_lock);
        if (!motor_controller_write_lck.try_lock()) {
          throw std::runtime_error(
            "Can't acquire mutex for writing commands - estop is being triggered");
        }

        motors_controller_->SendSpeedCommands(
          hw_commands_velocities_[0], hw_commands_velocities_[1], hw_commands_velocities_[2],
          hw_commands_velocities_[3]);
      }

      roboteq_error_filter_->UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, false);
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(logger_, "Error when trying to write commands: " << e.what());
      roboteq_error_filter_->UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);
    }
  }

  return return_type::OK;
}

bool PantherSystem::CheckIfSafetyStopActive()
{
  const auto & front_data = motors_controller_->GetFrontData();
  const auto & rear_data = motors_controller_->GetRearData();
  return front_data.GetLeftRuntimeError().GetMessage().safety_stop_active &&
         front_data.GetRightRuntimeError().GetMessage().safety_stop_active &&
         rear_data.GetLeftRuntimeError().GetMessage().safety_stop_active &&
         rear_data.GetRightRuntimeError().GetMessage().safety_stop_active;
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

void PantherSystem::SetEStop()
{
  RCLCPP_INFO(logger_, "Setting estop");
  bool gpio_controller_error = false;

  try {
    gpio_controller_->EStopTrigger();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Error when trying to set safety stop using GPIO: " << e.what());
    gpio_controller_error = true;
  }

  try {
    {
      std::lock_guard<std::mutex> lck_g(motor_controller_write_mtx_);
      motors_controller_->TurnOnSafetyStop();
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(
      logger_, "Error when trying to set safety stop using CAN command: " << e.what());
    if (gpio_controller_error) {
      RCLCPP_ERROR_STREAM(logger_, "Both attempts at setting estop failed");
      throw std::runtime_error("Both attempts at setting estop failed");
    }
  }

  estop_ = true;
}

void PantherSystem::ResetEStop()
{
  RCLCPP_INFO(logger_, "Resetting estop");

  // On side of the motors controller safety stop is reset by sending 0.0 commands
  if (!last_commands_zero_) {
    RCLCPP_ERROR(
      logger_,
      "Can't reset estop - last velocity commands are different than zero. Make sure that your "
      "controller sends zero commands before trying to reset estop.");
    throw std::runtime_error(
      "Can't reset estop - last velocity commands are different than zero. Make sure that your "
      "controller sends zero commands before trying to reset estop.");
  }

  try {
    gpio_controller_->EStopReset();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Error when trying to reset estop using GPIO: " << e.what());
    throw e;
  }

  roboteq_error_filter_->SetClearErrorsFlag();
  estop_ = false;
}

bool PantherSystem::ReadEStop()
{
  if (panther_version_ >= 1.2 - std::numeric_limits<float>::epsilon()) {
    // TODO: it has reversed logic
    return !gpio_controller_->IsPinActive(panther_gpiod::GPIOPin::E_STOP_RESET);
  } else {
    // For older panther versions there is no hardware EStop
    return estop_;
  }
}

bool PantherSystem::AreVelocityCommandsNearZero()
{
  for (const auto & cmd : hw_commands_velocities_) {
    if (std::abs(cmd) > std::numeric_limits<double>::epsilon()) {
      return false;
    }
  }
  return true;
}

}  // namespace panther_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  panther_hardware_interfaces::PantherSystem, hardware_interface::SystemInterface)
