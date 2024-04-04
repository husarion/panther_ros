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

#ifndef PANTHER_BATTERY_BATTERY_NODE_HPP_
#define PANTHER_BATTERY_BATTERY_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "panther_msgs/msg/driver_state.hpp"

#include "panther_battery/adc_data_reader.hpp"
#include "panther_battery/battery.hpp"
#include "panther_battery/battery_publisher.hpp"

namespace panther_battery
{

using DriverStateMsg = panther_msgs::msg::DriverState;

class BatteryNode : public rclcpp::Node
{
public:
  BatteryNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void BatteryPubTimerCB();
  void Initialize();
  void InitializeWithADCBattery();
  void InitializeWithRoboteqBattery();

  static constexpr int kADCCurrentOffset = 625;
  DriverStateMsg::SharedPtr driver_state_;

  std::shared_ptr<ADCDataReader> adc0_reader_;
  std::shared_ptr<ADCDataReader> adc1_reader_;
  std::shared_ptr<Battery> battery_1_;
  std::shared_ptr<Battery> battery_2_;
  std::shared_ptr<BatteryPublisher> battery_publisher_;

  rclcpp::Subscription<DriverStateMsg>::SharedPtr driver_state_sub_;
  rclcpp::TimerBase::SharedPtr battery_pub_timer_;

  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_BATTERY_NODE_HPP_
