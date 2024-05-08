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

#include "panther_battery/battery_publisher.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

namespace panther_battery
{

BatteryPublisher::BatteryPublisher(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater)
: node_(std::move(node)), diagnostic_updater_(std::move(diagnostic_updater))
{
  node_->declare_parameter<float>("battery_timeout", 1.0);
  battery_timeout_ = node_->get_parameter("battery_timeout").as_double();

  charger_connected_ = false;
  last_battery_info_time_ = rclcpp::Time(std::int64_t(0), RCL_ROS_TIME);

  io_state_sub_ = node_->create_subscription<IOStateMsg>(
    "hardware/io_state", 3,
    [&](const IOStateMsg::SharedPtr msg) { charger_connected_ = msg->charger_connected; });

  diagnostic_updater_->add("Battery errors", this, &BatteryPublisher::DiagnoseErrors);
  diagnostic_updater_->add("Battery status", this, &BatteryPublisher::DiagnoseStatus);
}

void BatteryPublisher::Publish()
{
  try {
    this->Update();
    last_battery_info_time_ = node_->get_clock()->now();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000, "Error reading battery data: %s. ", e.what());

    diagnostic_updater_->broadcast(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Error reading battery data: " + std::string(e.what()));
  }

  if (TimeoutReached()) {
    this->Reset();
  }

  this->PublishBatteryState();
  this->LogErrors();
}

bool BatteryPublisher::TimeoutReached() const
{
  return (node_->get_clock()->now() - last_battery_info_time_) >
         rclcpp::Duration::from_seconds(battery_timeout_);
}

void BatteryPublisher::BatteryStatusLogger(const BatteryStateMsg & battery_state) const
{
  std::string msg{};

  switch (battery_state.power_supply_status) {
    case BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING:
      msg =
        "The charger has been plugged in, but the charging process has not started. Check if the "
        "charger is connected to a power source.";

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, msg.c_str());
      break;

    case BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING:
      msg = "The robot is charging. Current battery percentage: " +
            std::to_string(static_cast<int>(round(battery_state.percentage * 100.0))) + "%.";

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 180000, msg.c_str());
      break;

    case BatteryStateMsg::POWER_SUPPLY_STATUS_FULL:
      msg = "The battery is fully charged. Robot can be disconnected from the charger.";

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 180000, msg.c_str());
      break;

    default:
      break;
  }
}

bool BatteryPublisher::ChargerConnected() const { return charger_connected_; }

std::string BatteryPublisher::MapPowerSupplyStatusToString(uint8_t power_supply_status) const
{
  switch (power_supply_status) {
    case BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN:
      return "Unknown";
    case BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING:
      return "Charging";
    case BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING:
      return "Discharging";
    case BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING:
      return "Plugged and not charging";
    case BatteryStateMsg::POWER_SUPPLY_STATUS_FULL:
      return "Battery full";
    default:
      return "Invalid status";
  }
}

}  // namespace panther_battery
