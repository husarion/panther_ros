#include <panther_hardware_interfaces/panther_system.hpp>

#include <rclcpp/logging.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

// TODO: add user variable to script that will trigger DOUT4 instead of safety stop

namespace panther_hardware_interfaces
{

using namespace std::literals;

void PantherSystem::CheckJointSize()
{
  if (info_.joints.size() != kJointsSize) {
    throw std::runtime_error(
      "Wrong number of joints defined: "s + std::to_string(info_.joints.size()) + ", "s +
      std::to_string(kJointsSize) + "expected."s);
  }
}

void PantherSystem::SortJointNames()
{
  // Sort joints names - later hw_states and hw_commands are accessed by static indexes, so it
  // is necessary to make sure that joints are in specific order and order of definitions in URDF
  // doesn't matter
  for (std::size_t i = 0; i < kJointsSize; i++) {
    for (std::size_t j = 0; j < kJointsSize; j++) {
      if (info_.joints[j].name.find(joint_order_[i]) != std::string::npos) {
        joints_names_sorted_[i] = info_.joints[j].name;
      }
    }
  }
}

void PantherSystem::CheckJointNames()
{
  for (std::size_t i = 0; i < kJointsSize; i++) {
    if (joints_names_sorted_[i] == "") {
      throw std::runtime_error(
        joint_order_[i] +
        " joint not defined (exactly one joint containing this string is required)"s);
    }
  }
}

void PantherSystem::SetInitialValues()
{
  // It isn't safe to set command to NaN - sometimes it could be interpreted as Inf (although it shouldn't)
  // In case of velocity, I think that setting initial value to 0.0 is the best option
  for (std::size_t i = 0; i < kJointsSize; i++) {
    hw_commands_velocities_[i] = 0.0;
    hw_states_positions_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_velocities_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_efforts_[i] = std::numeric_limits<double>::quiet_NaN();
  }
}

void PantherSystem::CheckInterfaces()
{
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // Commands
    if (joint.command_interfaces.size() != 1) {
      throw std::runtime_error(
        "Joint "s + joint.name + " has "s + std::to_string(joint.command_interfaces.size()) +
        " command interfaces found. 1 expected."s);
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      throw std::runtime_error(
        "Joint "s + joint.name + " have "s + joint.command_interfaces[0].name +
        " command interfaces found. "s + hardware_interface::HW_IF_VELOCITY + " expected."s);
    }

    // States
    if (joint.state_interfaces.size() != 3) {
      throw std::runtime_error(
        "Joint "s + joint.name + " has "s + std::to_string(joint.state_interfaces.size()) +
        " state interface. 3 expected."s);
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      throw std::runtime_error(
        "Joint "s + joint.name + " have "s + joint.state_interfaces[0].name +
        " as first state interface. "s + hardware_interface::HW_IF_POSITION + " expected."s);
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      throw std::runtime_error(
        "Joint "s + joint.name + " have "s + joint.state_interfaces[1].name +
        " as second state interface. "s + hardware_interface::HW_IF_VELOCITY + " expected."s);
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      throw std::runtime_error(
        "Joint "s + joint.name + " have "s + joint.state_interfaces[2].name +
        " as third state interface. "s + hardware_interface::HW_IF_EFFORT + " expected."s);
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

void PantherSystem::ReadCanOpenSettings()
{
  canopen_settings_.master_can_id = std::stoi(info_.hardware_parameters["master_can_id"]);
  canopen_settings_.front_driver_can_id =
    std::stoi(info_.hardware_parameters["front_driver_can_id"]);
  canopen_settings_.rear_driver_can_id = std::stoi(info_.hardware_parameters["rear_driver_can_id"]);
  canopen_settings_.pdo_feedback_timeout =
    std::chrono::milliseconds(std::stoi(info_.hardware_parameters["pdo_feedback_timeout"]));
  canopen_settings_.sdo_operation_timeout =
    std::chrono::milliseconds(std::stoi(info_.hardware_parameters["sdo_operation_timeout"]));
}

void PantherSystem::ReadInitializationActivationAttempts()
{
  max_roboteq_initialization_attempts_ =
    std::stoi(info_.hardware_parameters["roboteq_initialization_attempts"]);
  max_roboteq_activation_attempts_ =
    std::stoi(info_.hardware_parameters["roboteq_activation_attempts"]);
}

void PantherSystem::ReadParametersAndCreateCanOpenErrorFilter()
{
  unsigned max_write_sdo_errors_count =
    std::stoi(info_.hardware_parameters["max_write_sdo_errors_count"]);
  unsigned max_read_sdo_errors_count =
    std::stoi(info_.hardware_parameters["max_read_sdo_errors_count"]);
  unsigned max_read_pdo_errors_count =
    std::stoi(info_.hardware_parameters["max_read_pdo_errors_count"]);

  canopen_error_filter_ = std::make_shared<CanOpenErrorFilter>(
    max_write_sdo_errors_count, max_read_sdo_errors_count, max_read_pdo_errors_count);
}

CallbackReturn PantherSystem::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Initializing");

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    CheckJointSize();
    SortJointNames();
    CheckJointNames();
    SetInitialValues();
    CheckInterfaces();
  } catch (std::runtime_error & err) {
    RCLCPP_FATAL_STREAM(
      rclcpp::get_logger("PantherSystem"), "Exception during initialization: " << err.what());
    return CallbackReturn::ERROR;
  }

  try {
    ReadDrivetrainSettings();
    ReadCanOpenSettings();
    ReadInitializationActivationAttempts();
    ReadParametersAndCreateCanOpenErrorFilter();

  } catch (std::invalid_argument & err) {
    RCLCPP_FATAL(
      rclcpp::get_logger("PantherSystem"),
      "One of the required hardware parameters was not defined");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Configuring");

  roboteq_controller_ =
    std::make_shared<PantherWheelsController>(canopen_settings_, drivetrain_settings_);

  // Waiting for final GPIO implementation, current one doesn't work due to permission issues
  // gpio_controller_ = std::make_unique<GPIOController>();

  panther_system_node_.Configure();

  // TODO: add tests

  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Initializing roboteqs");

  if (!OperationWithAttempts(
        std::bind(&PantherWheelsController::Initialize, roboteq_controller_),
        max_roboteq_initialization_attempts_,
        std::bind(&PantherWheelsController::Deinitialize, roboteq_controller_))) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("PantherSystem"), "Initialization failed");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Cleaning up");

  roboteq_controller_->Deinitialize();
  roboteq_controller_.reset();

  panther_system_node_.DestroyNode();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Activating");

  for (std::size_t i = 0; i < kJointsSize; i++) {
    hw_commands_velocities_[i] = 0.0;
    hw_states_positions_[i] = 0.0;
    hw_states_velocities_[i] = 0.0;
    hw_states_efforts_[i] = 0.0;
  }

  // gpio_controller_->start();

  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Activating roboteqs");

  if (!OperationWithAttempts(
        std::bind(&PantherWheelsController::Activate, roboteq_controller_),
        max_roboteq_activation_attempts_, []() {})) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("PantherSystem"), "Activation failed");
    return CallbackReturn::FAILURE;
  }

  panther_system_node_.Activate(
    std::bind(&CanOpenErrorFilter::SetClearErrorsFlag, canopen_error_filter_));

  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Activation finished");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Deactivating");

  try {
    roboteq_controller_->TurnOnSafetyStop();
  } catch (std::runtime_error & err) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("PantherSystem"), "on_error failure " << err.what());
    return CallbackReturn::FAILURE;
  }

  panther_system_node_.ResetPublishers();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Shutting down");
  try {
    roboteq_controller_->TurnOnSafetyStop();
  } catch (std::runtime_error & err) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("PantherSystem"), "on_error failure " << err.what());
    return CallbackReturn::FAILURE;
  }

  panther_system_node_.ResetPublishers();

  roboteq_controller_->Deinitialize();
  roboteq_controller_.reset();

  panther_system_node_.DestroyNode();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Handling error");

  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Setting safe stop");
  if (!OperationWithAttempts(
        std::bind(&PantherWheelsController::TurnOnSafetyStop, roboteq_controller_),
        max_safety_stop_attempts_, []() {})) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("PantherSystem"), "safety stop failed");
    return CallbackReturn::FAILURE;
  }

  panther_system_node_.ResetPublishers();

  roboteq_controller_->Deinitialize();
  roboteq_controller_.reset();

  panther_system_node_.DestroyNode();

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> PantherSystem::export_state_interfaces()
{
  // TODO: check order
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
    // were read, sent feedback message is updated. As there are 8 values, frequency of updates will
    // be controller_frequency / 8
    bool finished_updates = roboteq_controller_->UpdateDriversState();

    if (finished_updates) {
      // TODO: locking???
      panther_system_node_.UpdateMsgDriversState(
        roboteq_controller_->GetFrontData().GetDriverState(),
        roboteq_controller_->GetRearData().GetDriverState());
    }

    canopen_error_filter_->UpdateReadSDOError(false);

  } catch (std::runtime_error & err) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("PantherSystem"),
      "Error when trying to read drivers feedback: " << err.what());
    canopen_error_filter_->UpdateReadSDOError(true);
  }
}

void PantherSystem::UpdateSystemFeedback()
{
  try {
    roboteq_controller_->UpdateSystemFeedback();
    UpdateHwStates();
    panther_system_node_.UpdateMsgDriversErrorsState(
      roboteq_controller_->GetFrontData(), roboteq_controller_->GetRearData());

    if (
      roboteq_controller_->GetFrontData().IsError() ||
      roboteq_controller_->GetRearData().IsError()) {
      // TODO: fix
      // RCLCPP_ERROR_STREAM_THROTTLE(
      //   rclcpp::get_logger("PantherSystem"), *node_->get_clock(), 5000,
      //   "Error state on one of the drivers"
      //     << "\nFront: " << roboteq_controller_->GetFrontData().GetErrorLog()
      //     << "\nRear: " << roboteq_controller_->GetRearData().GetErrorLog());

      // TODO: filter only on timeout errors
      canopen_error_filter_->UpdateReadPDOError(true);
    } else {
      canopen_error_filter_->UpdateReadPDOError(false);
    }

  } catch (std::runtime_error & err) {
    canopen_error_filter_->UpdateReadPDOError(true);
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("PantherSystem"), "Error when trying to read feedback: " << err.what());
  }
}

return_type PantherSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  UpdateDriverState();
  UpdateSystemFeedback();
  panther_system_node_.UpdateMsgErrors(
    canopen_error_filter_->IsError(), canopen_error_filter_->IsWriteSDOError(),
    canopen_error_filter_->IsReadSDOError(), canopen_error_filter_->IsReadPDOError(),
    roboteq_controller_->GetFrontData().GetOldData(),
    roboteq_controller_->GetRearData().GetOldData());

  panther_system_node_.PublishDriverState();

  return return_type::OK;
}

return_type PantherSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // "soft" error - still there is a comunication over CAN with drivers, so publishing feedback is continued -
  // hardware interface's onError isn't triggered
  // estop is handled similarly - at the time of writing there wasn't better approach to handle estop
  if (canopen_error_filter_->IsError()) {
    if (
      (roboteq_controller_->GetFrontData().GetLeftRuntimeError().GetMessage().safety_stop_active &&
       roboteq_controller_->GetFrontData().GetRightRuntimeError().GetMessage().safety_stop_active &&
       roboteq_controller_->GetRearData().GetLeftRuntimeError().GetMessage().safety_stop_active &&
       roboteq_controller_->GetRearData().GetRightRuntimeError().GetMessage().safety_stop_active) ==
      false) {
      RCLCPP_ERROR(rclcpp::get_logger("PantherSystem"), "Sending safety stop request");
      // 0 command is set with safety stop
      try {
        roboteq_controller_->TurnOnSafetyStop();
      } catch (std::runtime_error & err) {
        RCLCPP_FATAL_STREAM(
          rclcpp::get_logger("PantherSystem"),
          "Error when trying to turn on safety stop: " << err.what());
        return return_type::ERROR;
      }
    }

    // TODO: fix
    // RCLCPP_ERROR_STREAM_THROTTLE(
    //   rclcpp::get_logger("PantherSystem"), *node_->get_clock(), 5000,
    //   "Error detected, ignoring write commands");
    return return_type::OK;
  }

  try {
    roboteq_controller_->WriteSpeed(
      hw_commands_velocities_[0], hw_commands_velocities_[1], hw_commands_velocities_[2],
      hw_commands_velocities_[3]);
    canopen_error_filter_->UpdateWriteSDOError(false);
  } catch (std::runtime_error & err) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("PantherSystem"), "Error when trying to write commands: " << err.what());
    canopen_error_filter_->UpdateWriteSDOError(true);
  }

  return return_type::OK;
}

void PantherSystem::UpdateHwStates()
{
  auto front = roboteq_controller_->GetFrontData();
  auto rear = roboteq_controller_->GetRearData();

  auto fl = front.GetLeftMotorState();
  auto fr = front.GetRightMotorState();
  auto rl = rear.GetLeftMotorState();
  auto rr = rear.GetRightMotorState();

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