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

#ifndef PANTHER_BATTERY_DUAL_BATTERY_PUBLISHER_HPP_
#define PANTHER_BATTERY_DUAL_BATTERY_PUBLISHER_HPP_

#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "panther_battery/battery.hpp"
#include "panther_battery/battery_publisher.hpp"

namespace panther_battery
{

class DualBatteryPublisher : public BatteryPublisher
{
public:
  DualBatteryPublisher(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
    const std::shared_ptr<Battery> & battery_1, const std::shared_ptr<Battery> & battery_2);

  ~DualBatteryPublisher() {}

protected:
  void Update() override;
  void Reset() override;
  void PublishBatteryState() override;
  void LogErrors() override;
  void DiagnoseBattery(diagnostic_updater::DiagnosticStatusWrapper & status) override;

  BatteryStateMsg MergeBatteryMsgs(
    const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2);
  std::uint8_t MergeBatteryPowerSupplyStatus(
    const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2) const;
  void MergeBatteryPowerSupplyHealth(
    BatteryStateMsg & battery_msg, const BatteryStateMsg & battery_msg_1,
    const BatteryStateMsg & battery_msg_2);

private:
  std::shared_ptr<Battery> battery_1_;
  std::shared_ptr<Battery> battery_2_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_1_pub_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_2_pub_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_DUAL_BATTERY_PUBLISHER_HPP_
