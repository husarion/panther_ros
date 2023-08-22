#include <panther_hardware_interfaces/panther_system.hpp>

#include <rclcpp/logging.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

// TODO use safety stop instead of estop
// TODO: add user variable to script that will trigger DOUT4 instead of safety stop

namespace panther_hardware_interfaces
{
CallbackReturn PantherSystem::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Initializing");

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointsSize) {
    RCLCPP_FATAL(
      rclcpp::get_logger("PantherSystem"), "Wrong number of joints defined: %zu, %zu expected.",
      info_.joints.size(), kJointsSize);
    return CallbackReturn::ERROR;
  }

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

  for (std::size_t i = 0; i < kJointsSize; i++) {
    if (joints_names_sorted_[i] == "") {
      RCLCPP_FATAL(
        rclcpp::get_logger("PantherSystem"),
        "%s joint not defined (exactly one joint containing this string is required)",
        joint_order_[i].c_str());
      return CallbackReturn::ERROR;
    }
  }

  // It isn't safe to set command to NaN - sometimes it could be interpreted as Inf (although it shouldn't)
  // In case of velocity, I think that setting initial value to 0.0 is the best option
  for (std::size_t i = 0; i < kJointsSize; i++) {
    hw_commands_velocities_[i] = 0.0;
    hw_states_positions_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_velocities_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_efforts_[i] = std::numeric_limits<double>::quiet_NaN();
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // Commands
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("PantherSystem"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("PantherSystem"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    // States
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("PantherSystem"), "Joint '%s' has %zu state interface. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("PantherSystem"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("PantherSystem"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger("PantherSystem"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  try {
    drivetrain_settings_.motor_torque_constant =
      std::stof(info_.hardware_parameters["motor_torque_constant"]);
    drivetrain_settings_.gear_ratio = std::stof(info_.hardware_parameters["gear_ratio"]);
    drivetrain_settings_.gearbox_efficiency =
      std::stof(info_.hardware_parameters["gearbox_efficiency"]);
    drivetrain_settings_.encoder_resolution =
      std::stof(info_.hardware_parameters["encoder_resolution"]);
    drivetrain_settings_.max_rpm_motor_speed =
      std::stof(info_.hardware_parameters["max_rpm_motor_speed"]);

    can_settings_.master_can_id = std::stoi(info_.hardware_parameters["master_can_id"]);
    can_settings_.front_driver_can_id = std::stoi(info_.hardware_parameters["front_driver_can_id"]);
    can_settings_.rear_driver_can_id = std::stoi(info_.hardware_parameters["rear_driver_can_id"]);

    roboteq_state_period_ = std::stof(info_.hardware_parameters["roboteq_state_period"]);
  } catch (std::invalid_argument & err) {
    RCLCPP_FATAL(
      rclcpp::get_logger("PantherSystem"),
      "One of the required hardware parameters was not defined");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void PantherSystem::reset_publishers()
{
  realtime_driver_state_publisher_.reset();
  driver_state_publisher_.reset();
}

void PantherSystem::destroy_node()
{
  roboteq_controller_->Deinitialize();

  stop_executor_.store(true);
  // TODO: check
  executor_thread_->join();
  stop_executor_.store(false);

  executor_.reset();
  node_.reset();
  roboteq_controller_.reset();
}

CallbackReturn PantherSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Configuring");

  roboteq_controller_ =
    std::make_unique<PantherWheelsController>(can_settings_, drivetrain_settings_);

  // Waiting for final GPIO implementation, current one doesn't work due to permission issues
  // gpio_controller_ = std::make_unique<GPIOController>();

  node_ = std::make_shared<rclcpp::Node>("panther_system_node");
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  executor_thread_ = std::make_unique<std::thread>([this]() {
    while (!stop_executor_) {
      executor_->spin_some();
    }
  });

  next_roboteq_state_update_ = node_->get_clock()->now();

  try {
    roboteq_controller_->Initialize();
  } catch (std::runtime_error & err) {
    RCLCPP_FATAL(rclcpp::get_logger("PantherSystem"), "Initialization failed");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Cleaning up");

  destroy_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Activating");

  // TODO
  try {
    roboteq_controller_->TurnOffEstop();
  } catch (std::runtime_error & err) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("PantherSystem"), "Activation failed " << err.what());
    return CallbackReturn::FAILURE;
  }

  for (std::size_t i = 0; i < kJointsSize; i++) {
    hw_commands_velocities_[i] = 0.0;
    hw_states_positions_[i] = 0.0;
    hw_states_velocities_[i] = 0.0;
    hw_states_efforts_[i] = 0.0;
  }

  // gpio_controller_->start();

  try {
    roboteq_controller_->Activate();
  } catch (std::runtime_error & err) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("PantherSystem"), "Activation failed " << err.what());
    return CallbackReturn::FAILURE;
  }

  driver_state_publisher_ = node_->create_publisher<panther_msgs::msg::DriverState>(
    "~/driver/motor_controllers_state", rclcpp::SensorDataQoS());
  realtime_driver_state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<panther_msgs::msg::DriverState>>(
      driver_state_publisher_);

  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Activation finished");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Deactivating");

  // TODO maybe send 0 command first
  // roboteq_controller_->Deactivate();
  reset_publishers();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Shutting down");
  // TODO
  // roboteq_controller_->Deinitialize();
  reset_publishers();
  destroy_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Handling error");

  // TODO
  // Called when error is return from read or write
  // Maybe trigger estop?
  try {
    roboteq_controller_->TurnOnEstop();
  } catch (std::runtime_error & err) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("PantherSystem"), "on_error failure " << err.what());
    return CallbackReturn::FAILURE;
  }
  // TODO
  // roboteq_controller_->Deinitialize();

  reset_publishers();
  destroy_node();
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

return_type PantherSystem::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  // TODO!!!! add reading other stuff

  if (time > next_roboteq_state_update_) {
    try {
      roboteq_controller_->UpdateDriversState();

      // TODO: locking???
      auto & driver_state = realtime_driver_state_publisher_->msg_;

      auto front = roboteq_controller_->GetFrontData().GetDriverState();
      auto rear = roboteq_controller_->GetRearData().GetDriverState();

      driver_state.front.voltage = front.GetVoltage();
      driver_state.front.current = front.GetCurrent();
      driver_state.front.temperature = front.GetTemperature();

      driver_state.rear.voltage = rear.GetVoltage();
      driver_state.rear.current = rear.GetCurrent();
      driver_state.rear.temperature = rear.GetTemperature();

      next_roboteq_state_update_ = time + rclcpp::Duration::from_seconds(roboteq_state_period_);
    } catch (std::runtime_error & err) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("PantherSystem"),
        "Error when trying to read drivers feedback: " << err.what());
      return return_type::ERROR;
    }
  }

  try {
    roboteq_controller_->UpdateSystemFeedback();

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

    auto & driver_state = realtime_driver_state_publisher_->msg_;

    driver_state.front.fault_flag = front.GetFaultFlag().GetMessage();
    driver_state.front.script_flag = front.GetScriptFlag().GetMessage();
    driver_state.front.left_motor.runtime_error = front.GetLeftRuntimeError().GetMessage();
    driver_state.front.right_motor.runtime_error = front.GetRightRuntimeError().GetMessage();

    driver_state.rear.fault_flag = rear.GetFaultFlag().GetMessage();
    driver_state.rear.script_flag = rear.GetScriptFlag().GetMessage();
    driver_state.rear.left_motor.runtime_error = rear.GetLeftRuntimeError().GetMessage();
    driver_state.rear.right_motor.runtime_error = rear.GetRightRuntimeError().GetMessage();

    // TODO old data to message

    error_ = front.IsError() || rear.IsError();

  } catch (std::runtime_error & err) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("PantherSystem"), "Error when trying to read feedback: " << err.what());
    return return_type::ERROR;
  }

  if (realtime_driver_state_publisher_->trylock()) {
    realtime_driver_state_publisher_->unlockAndPublish();
  }

  return return_type::OK;
}

return_type PantherSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // "soft" error - still there is a comunication over CAN with drivers, so publishing feedback is continued -
  // hardware interface's onError isn't triggered
  // estop is handled similarly - at the time of writing there wasn't better approach to handle estop
  if (error_) {
    return return_type::OK;
  }

  try {
    roboteq_controller_->WriteSpeed(
      hw_commands_velocities_[0], hw_commands_velocities_[1], hw_commands_velocities_[2],
      hw_commands_velocities_[3]);
  } catch (std::runtime_error & err) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("PantherSystem"), "Error when trying to write commands: " << err.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

}  // namespace panther_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  panther_hardware_interfaces::PantherSystem, hardware_interface::SystemInterface)