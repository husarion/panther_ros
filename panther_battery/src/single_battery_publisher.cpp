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

#include "panther_battery/single_battery_publisher.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "panther_battery/battery.hpp"
#include "panther_battery/battery_publisher.hpp"

namespace panther_battery
{

SingleBatteryPublisher::SingleBatteryPublisher(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
  const std::shared_ptr<Battery> & battery)
: BatteryPublisher(std::move(node), std::move(diagnostic_updater)), battery_(std::move(battery))
{
  battery_pub_ = node_->create_publisher<BatteryStateMsg>("battery", 5);
  battery_1_pub_ = node_->create_publisher<BatteryStateMsg>("battery_1_raw", 5);
}

void SingleBatteryPublisher::Update()
{
  const auto header_stamp = node_->get_clock()->now();
  battery_->Update(header_stamp, ChargerConnected());
}

void SingleBatteryPublisher::Reset()
{
  const auto header_stamp = node_->get_clock()->now();
  battery_->Reset(header_stamp);
}

void SingleBatteryPublisher::PublishBatteryState()
{
  const auto battery_msg = battery_->GetBatteryMsg();
  battery_pub_->publish(battery_msg);
  battery_1_pub_->publish(battery_->GetBatteryMsgRaw());
  BatteryStatusLogger(battery_msg);
}

void SingleBatteryPublisher::LogErrors()
{
  if (battery_->HasErrorMsg()) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000, "Battery error: %s",
      battery_->GetErrorMsg().c_str());
  }
}

void SingleBatteryPublisher::DiagnoseBattery(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char error_level{diagnostic_updater::DiagnosticStatusWrapper::OK};
  std::string message{"Battery has no error messages"};

  if (battery_->HasErrorMsg()) {
    error_level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "Battery has error";

    status.add("Error message", battery_->GetErrorMsg());
  }

  status.summary(error_level, message);
}

}  // namespace panther_battery
