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

#ifndef PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_
#define PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_

#include <memory>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "panther_msgs/msg/io_state.hpp"

namespace panther_battery
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using IOStateMsg = panther_msgs::msg::IOState;

class BatteryPublisher
{
public:
  BatteryPublisher(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater);

  ~BatteryPublisher() {}

  void Publish();

protected:
  virtual void Update() = 0;
  virtual void Reset() = 0;
  virtual void PublishBatteryState() = 0;
  virtual void LogErrors() = 0;
  virtual void DiagnoseBattery(diagnostic_updater::DiagnosticStatusWrapper & status) = 0;

  bool TimeoutReached() const;
  void BatteryStatusLogger(const BatteryStateMsg & battery_state) const;
  bool ChargerConnected() const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;

private:
  bool charger_connected_;
  float battery_timeout_;
  rclcpp::Time last_battery_info_time_;
  rclcpp::Subscription<IOStateMsg>::SharedPtr io_state_sub_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_
