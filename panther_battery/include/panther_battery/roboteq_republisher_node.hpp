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

#ifndef PANTHER_BATTERY_ROBOTEQ_REPUBLISHER_NODE_HPP_
#define PANTHER_BATTERY_ROBOTEQ_REPUBLISHER_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <panther_utils/moving_average.hpp>

namespace panther_battery
{
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using DriverStateMsg = panther_msgs::msg::DriverState;

class RoboteqRepublisherNode : public rclcpp::Node
{
public:
  RoboteqRepublisherNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void MotorControllersStateSubCB(const DriverStateMsg::SharedPtr msg);
  void BatteryPubTimerCB();

  static constexpr float kBatDesignedCapacity = 20.0;
  static constexpr float kVBatFatalMin = 27.0;
  static constexpr float kVBatFatalMax = 43.0;
  static constexpr float kVBatFull = 41.4;
  static constexpr float kVBatMin = 32.0;
  static constexpr std::string_view kLocation = "user_compartment";

  float battery_timeout_;
  rclcpp::Time last_battery_info_time_;

  std::unique_ptr<panther_utils::MovingAverage<double>> battery_voltage_ma_;
  std::unique_ptr<panther_utils::MovingAverage<double>> battery_current_ma_;

  rclcpp::Subscription<DriverStateMsg>::SharedPtr motor_controllers_state_sub_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr battery_pub_timer_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ROBOTEQ_REPUBLISHER_NODE_HPP_
