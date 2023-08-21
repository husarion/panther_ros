#ifndef PANTHER_HARDWARE_INTERFACES__TEST_UTILS_HPP_
#define PANTHER_HARDWARE_INTERFACES__TEST_UTILS_HPP_

#include <string>

#include <lifecycle_msgs/msg/state.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

const std::string panther_system_name = "wheels";

const std::string panther_system_urdf =
  R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="Panther">
  <ros2_control name="wheels" type="system">
    <hardware>
      <plugin>panther_hardware_interfaces/PantherSystem</plugin>
      <param name="encoder_resolution">1600</param>
      <param name="gear_ratio">30.08</param>
      <param name="gearbox_efficiency">0.75</param>
      <param name="motor_torque_constant">0.11</param>
      <param name="max_rpm_motor_speed">3600.0</param>
      <param name="master_can_id">3</param>
      <param name="front_driver_can_id">1</param>
      <param name="rear_driver_can_id">2</param>
      <param name="roboteq_state_period">1.0</param>
    </hardware>

    <joint name="fl_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
    <joint name="fr_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
    <joint name="rl_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
    <joint name="rr_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
  </ros2_control>
</robot>
)";

void set_components_state(
  hardware_interface::ResourceManager & rm, const std::vector<std::string> & components,
  const uint8_t state_id, const std::string & state_name)
{
  for (const auto & component : components) {
    rclcpp_lifecycle::State state(state_id, state_name);
    rm.set_component_state(component, state);
  }
}

auto configure_components = [](
                              hardware_interface::ResourceManager & rm,
                              const std::vector<std::string> & components = {panther_system_name}) {
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
};

auto unconfigure_components =
  [](
    hardware_interface::ResourceManager & rm,
    const std::vector<std::string> & components = {panther_system_name}) {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
  };

auto activate_components = [](
                             hardware_interface::ResourceManager & rm,
                             const std::vector<std::string> & components = {panther_system_name}) {
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);
};

auto deactivate_components =
  [](
    hardware_interface::ResourceManager & rm,
    const std::vector<std::string> & components = {panther_system_name}) {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  };

auto shutdown_components = [](
                             hardware_interface::ResourceManager & rm,
                             const std::vector<std::string> & components = {panther_system_name}) {
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
    hardware_interface::lifecycle_state_names::FINALIZED);
};

#endif