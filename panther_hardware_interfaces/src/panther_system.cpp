#include <panther_hardware_interfaces/panther_system.hpp>

#include <rclcpp/logging.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace panther_hardware_interfaces
{
CallbackReturn PantherSystem::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Initializing");

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != JOINTS_SIZE_) {
    RCLCPP_FATAL(
      rclcpp::get_logger("PantherSystem"), "Wrong number of joints defined: %zu, %zu expected.",
      info_.joints.size(), JOINTS_SIZE_);
    return CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < JOINTS_SIZE_; i++) {
    for (std::size_t j = 0; j < JOINTS_SIZE_; j++) {
      if (info_.joints[j].name.find(joint_order_[i]) != std::string::npos) {
        joints_names_sorted_[i] = info_.joints[j].name;
      }
    }
  }

  for (std::size_t i = 0; i < JOINTS_SIZE_; i++) {
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
  for (std::size_t i = 0; i < JOINTS_SIZE_; i++) {
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

  // TODO add checking if parameters were defined
  DrivetrainSettings drivetrain_settings;
  drivetrain_settings.motor_torque_constant =
    std::stof(info_.hardware_parameters["motor_torque_constant"]);
  drivetrain_settings.gear_ratio = std::stof(info_.hardware_parameters["gear_ratio"]);
  drivetrain_settings.gearbox_efficiency =
    std::stof(info_.hardware_parameters["gearbox_efficiency"]);
  drivetrain_settings.encoder_resolution =
    std::stof(info_.hardware_parameters["encoder_resolution"]);
  drivetrain_settings.max_rpm_motor_speed =
    std::stof(info_.hardware_parameters["max_rpm_motor_speed"]);

  CanSettings can_settings;
  can_settings.master_can_id = std::stoi(info_.hardware_parameters["master_can_id"]);
  can_settings.front_driver_can_id = std::stoi(info_.hardware_parameters["front_driver_can_id"]);
  can_settings.rear_driver_can_id = std::stoi(info_.hardware_parameters["rear_driver_can_id"]);

  roboteq_controller_ =
    std::make_unique<PantherWheelsController>(can_settings, drivetrain_settings);
  // TODO comment
  // gpio_controller_ = std::make_unique<GPIOController>();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Configuring");

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
  // TODO maybe send 0 command first
  roboteq_controller_->Deinitialize();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Activating");

  for (std::size_t i = 0; i < JOINTS_SIZE_; i++) {
    hw_commands_velocities_[i] = 0.0;
    hw_states_positions_[i] = 0.0;
    hw_states_velocities_[i] = 0.0;
    hw_states_efforts_[i] = 0.0;
  }

  // gpio_controller_->start();

  try {
    roboteq_controller_->Activate();
  } catch (std::runtime_error & err) {
    RCLCPP_FATAL(rclcpp::get_logger("PantherSystem"), "Activation failed");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Activation finished");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Deactivating");

  // roboteq_controller_->Deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Shutting down");
  // TODO
  // roboteq_controller_->Deinitialize();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Handling error");
  // TODO
  // roboteq_controller_->Deinitialize();

  // TODO
  // Called when error is return from read or write
  // Maybe trigger estop?
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> PantherSystem::export_state_interfaces()
{
  // TODO: check order
  std::vector<StateInterface> state_interfaces;
  for (std::size_t i = 0; i < JOINTS_SIZE_; i++) {
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
  for (std::size_t i = 0; i < JOINTS_SIZE_; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joints_names_sorted_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

return_type PantherSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO!!!! add reading other stuff

  try {
    RoboteqFeedback feedback = roboteq_controller_->Read();

    hw_states_positions_[0] = feedback.pos_fl;
    hw_states_positions_[1] = feedback.pos_fr;
    hw_states_positions_[2] = feedback.pos_rl;
    hw_states_positions_[3] = feedback.pos_rr;

    hw_states_velocities_[0] = feedback.vel_fl;
    hw_states_velocities_[1] = feedback.vel_fr;
    hw_states_velocities_[2] = feedback.vel_rl;
    hw_states_velocities_[3] = feedback.vel_rr;

    hw_states_efforts_[0] = feedback.torque_fl;
    hw_states_efforts_[1] = feedback.torque_fr;
    hw_states_efforts_[2] = feedback.torque_rl;
    hw_states_efforts_[3] = feedback.torque_rr;
  } catch (std::runtime_error & err) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("PantherSystem"), "Error when trying to read feedback: " << err.what());
    return return_type::ERROR;
  }

  // TODO!!!!! error flags

  return return_type::OK;
}

return_type PantherSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
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