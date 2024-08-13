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

#include <memory>

#include "ign_ros2_control/ign_system.hpp"
#include "ign_ros2_control/ign_system_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace panther_gazebo
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// These class must inherit `ign_ros2_control::IgnitionSystemInterface` which implements a
// simulated `ros2_control` `hardware_interface::SystemInterface`.

/// \class GzPantherSystem
/// \brief Main class for the Panther System which inherits from IgnitionSystemInterface. Based on:
/// https://github.com/ros-controls/gz_ros2_control/blob/humble/ign_ros2_control/src/ign_system.cpp
class GzPantherSystem : public ign_ros2_control::IgnitionSystem
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool e_stop_active;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr e_stop_publisher;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr e_stop_reset_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr e_stop_trigger_service;

  void SetupEStop();

  void PublishEStopStatus();

  void EStopResetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void EStopTriggerCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

}  // namespace panther_gazebo

#endif  // PANTHER_GAZEBO_GZ_PANTHER_SYSTEM
