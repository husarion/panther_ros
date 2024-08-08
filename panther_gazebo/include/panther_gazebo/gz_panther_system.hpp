// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef PANTHER_GAZEBO_GZ_PANTHER_SYSTEM
#define PANTHER_GAZEBO_GZ_PANTHER_SYSTEM

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ign_ros2_control/ign_system_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace panther_gazebo
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Forward declaration
class PantherSystemPrivate;

// These class must inherit `ign_ros2_control::IgnitionSystemInterface` which implements a
// simulated `ros2_control` `hardware_interface::SystemInterface`.

/// \class PantherSystem
/// \brief Main class for the Panther System which inherits from IgnitionSystemInterface. Based on:
/// https://github.com/ros-controls/gz_ros2_control/blob/humble/ign_ros2_control/src/ign_system.cpp
class PantherSystem : public ign_ros2_control::IgnitionSystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Initializes the simulation environment for the Panther robot.
   *
   * This function sets up the internal state of the PantherSystem, configures the simulation
   * joints, and registers state and command interfaces based on the provided hardware information.
   * It also handles any mimicking of joints and sets the appropriate parameters for the simulation.
   *
   * @param model_nh Shared pointer to the ROS node handle.
   * @param enableJoints Map of joint names to their corresponding Gazebo entities.
   * @param hardware_info Struct containing hardware configuration details.
   * @param _ecm Entity Component Manager for managing Gazebo entities and components.
   * @param update_rate Reference to the simulation update rate.
   *
   * @return True if initialization is successful, false otherwise.
   */
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh, std::map<std::string, ignition::gazebo::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    ignition::gazebo::EntityComponentManager & _ecm, int & update_rate) override;

private:
  /// \brief Register a sensor (for now just IMUs)
  /// \param[in] hardware_info Hardware information containing sensor data.
  void registerSensors(const hardware_interface::HardwareInfo & hardware_info);

  void readParams(const hardware_interface::HardwareInfo & hardware_info);

  void setupEStop();

  void publishEStopStatus();

  void eStopResetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void eStopTriggerCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// \brief Private data class
  std::unique_ptr<PantherSystemPrivate> dataPtr;
};

}  // namespace panther_gazebo

#endif  // PANTHER_GAZEBO_GZ_PANTHER_SYSTEM
