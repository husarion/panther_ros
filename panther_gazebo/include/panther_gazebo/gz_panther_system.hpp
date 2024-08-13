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

#include <ign_ros2_control/ign_system.hpp>
#include <ign_ros2_control/ign_system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace panther_gazebo
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using BoolMsg = std_msgs::msg::Bool;
using TriggerSrv = std_srvs::srv::Trigger;

/// \class GzPantherSystem
/// \brief Main class for the Panther System which implements a
// simulated `ros2_control` `hardware_interface::SystemInterface`. These class must
/// inherit `ign_ros2_control::IgnitionSystemInterface`. Based on:
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
  bool e_stop_active_;
  rclcpp::Publisher<BoolMsg>::SharedPtr e_stop_publisher_;
  rclcpp::Service<TriggerSrv>::SharedPtr e_stop_reset_service_;
  rclcpp::Service<TriggerSrv>::SharedPtr e_stop_trigger_service_;

  void SetupEStop();

  void PublishEStopStatus();

  void EStopResetCallback(
    const TriggerSrv::Request::SharedPtr & request, TriggerSrv::Response::SharedPtr response);

  void EStopTriggerCallback(
    const TriggerSrv::Request::SharedPtr & request, TriggerSrv::Response::SharedPtr response);
};

}  // namespace panther_gazebo

#endif  // PANTHER_GAZEBO_GZ_PANTHER_SYSTEM
