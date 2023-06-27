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

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
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

  for (auto & j : info_.joints) {
    RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Joint '%s' found", j.name.c_str());

    pos_state_[j.name] = 0.0;
    vel_state_[j.name] = 0.0;
    effort_state_[j.name] = 0.0;

    vel_commands_[j.name] = 0.0;
    // pos_commands_[j.name] = 0.0;
    // effort_commands_[j.name] = 0.0;
  }

  RoboteqSettings settings;

  settings.motor_torque_constant = std::stof(info_.hardware_parameters["motor_torque_constant"]);
  settings.gear_ratio = std::stof(info_.hardware_parameters["gear_ratio"]);
  settings.gearbox_efficiency = std::stof(info_.hardware_parameters["gearbox_efficiency"]);
  settings.encoder_resolution = std::stof(info_.hardware_parameters["encoder_resolution"]);
  settings.max_rpm_motor_speed = std::stof(info_.hardware_parameters["max_rpm_motor_speed"]);
  settings.max_amps_motor_current = std::stof(info_.hardware_parameters["max_amps_motor_current"]);
  settings.master_can_id = std::stoi(info_.hardware_parameters["master_can_id"]);
  settings.front_driver_can_id = std::stoi(info_.hardware_parameters["front_driver_can_id"]);
  settings.rear_driver_can_id = std::stoi(info_.hardware_parameters["rear_driver_can_id"]);

  roboteq_controller_ = std::make_unique<RoboteqController>(settings);

  hardware_interface_type_ = hardware_interface::HW_IF_VELOCITY;

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Configuring");

  // gpio_controller_ = std::make_unique<GPIOController>();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Activating");

  for (const auto & x : pos_state_) {
    pos_state_[x.first] = 0.0;
    vel_state_[x.first] = 0.0;
    effort_state_[x.first] = 0.0;

    vel_commands_[x.first] = 0.0;
    // pos_commands_[x.first] = 0.0;
    // effort_commands_[x.first] = 0.0;
  }

  // gpio_controller_->start();
  roboteq_controller_->Activate();

  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Activation finished");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Deactivating");

  roboteq_controller_->Deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Shutting down");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherSystem::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Handling error");
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> PantherSystem::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_state_[info_.joints[i].name]));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_state_[info_.joints[i].name]));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
      &effort_state_[info_.joints[i].name]));
  }

  return state_interfaces;
}

std::vector<CommandInterface> PantherSystem::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &vel_commands_[info_.joints[i].name]));

    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_POSITION,
    //   &pos_commands_[info_.joints[i].name]));
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
    //   &effort_commands_[info_.joints[i].name]));
  }

  return command_interfaces;
}

// Disabled, please check explaination above prepare_command_mode_switch declaration
// return_type PantherSystem::prepare_command_mode_switch(
//   const std::vector<std::string> & start_interfaces,
//   const std::vector<std::string> & stop_interfaces)
// {
//   RCLCPP_INFO_STREAM(
//     rclcpp::get_logger("PantherSystem"), "Preparing mode change to " << hardware_interface_type_);
//   if (start_interfaces[0] == info_.joints[0].name + "/" + hardware_interface::HW_IF_EFFORT) {
//     hardware_interface_type_ = hardware_interface::HW_IF_EFFORT;
//   } else if (
//     start_interfaces[0] == info_.joints[0].name + "/" + hardware_interface::HW_IF_VELOCITY) {
//     hardware_interface_type_ = hardware_interface::HW_IF_VELOCITY;
//   } else if (
//     start_interfaces[0] == info_.joints[0].name + "/" + hardware_interface::HW_IF_POSITION) {
//     hardware_interface_type_ = hardware_interface::HW_IF_POSITION;
//   }

//   return return_type::OK;
// }

// return_type PantherSystem::perform_command_mode_switch(
//   const std::vector<std::string> &, const std::vector<std::string> &)
// {
//   RCLCPP_INFO_STREAM(
//     rclcpp::get_logger("PantherSystem"), "Performing mode change to " << hardware_interface_type_);

//   if (hardware_interface_type_ == hardware_interface::HW_IF_POSITION) {
//     roboteq_controller_->ChangeMode(RoboteqMode::POSITION);
//   } else if (hardware_interface_type_ == hardware_interface::HW_IF_VELOCITY) {
//     roboteq_controller_->ChangeMode(RoboteqMode::VELOCITY);
//   } else if (hardware_interface_type_ == hardware_interface::HW_IF_EFFORT) {
//     roboteq_controller_->ChangeMode(RoboteqMode::TORQUE);
//   }

//   return return_type::OK;
// }

return_type PantherSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  RoboteqFeedback feedback = roboteq_controller_->Read();

  pos_state_["fr_wheel_joint"] = feedback.pos_fr;
  pos_state_["fl_wheel_joint"] = feedback.pos_fl;
  vel_state_["fr_wheel_joint"] = feedback.vel_fr;
  vel_state_["fl_wheel_joint"] = feedback.vel_fl;
  effort_state_["fr_wheel_joint"] = feedback.torque_fr;
  effort_state_["fl_wheel_joint"] = feedback.torque_fl;
  pos_state_["rr_wheel_joint"] = feedback.pos_rr;
  pos_state_["rl_wheel_joint"] = feedback.pos_rl;
  vel_state_["rr_wheel_joint"] = feedback.vel_rr;
  vel_state_["rl_wheel_joint"] = feedback.vel_rl;
  effort_state_["rr_wheel_joint"] = feedback.torque_rr;
  effort_state_["rl_wheel_joint"] = feedback.torque_rl;

  // TODO error flags

  return return_type::OK;
}

return_type PantherSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (hardware_interface_type_ == hardware_interface::HW_IF_VELOCITY) {
    roboteq_controller_->WriteSpeed(
      vel_commands_["fl_wheel_joint"], vel_commands_["fr_wheel_joint"],
      vel_commands_["rl_wheel_joint"], vel_commands_["rr_wheel_joint"]);
  }

  // if (hardware_interface_type_ == hardware_interface::HW_IF_POSITION) {
  // }
  // if (hardware_interface_type_ == hardware_interface::HW_IF_EFFORT) {
  //   roboteq_controller_->WriteTorque(
  //     effort_commands_["fl_wheel_joint"], effort_commands_["fr_wheel_joint"],
  //     effort_commands_["rl_wheel_joint"], effort_commands_["rr_wheel_joint"]);
  // }

  return return_type::OK;
}

}  // namespace panther_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  panther_hardware_interfaces::PantherSystem, hardware_interface::SystemInterface)