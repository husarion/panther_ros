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

#ifndef PANTHER_BATTERY_SINGLE_BATTERY_PUBLISHER_HPP_
#define PANTHER_BATTERY_SINGLE_BATTERY_PUBLISHER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "panther_battery/battery.hpp"
#include "panther_battery/battery_publisher.hpp"

namespace panther_battery
{

class SingleBatteryPublisher : public BatteryPublisher
{
public:
  SingleBatteryPublisher(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
    const std::shared_ptr<Battery> & battery);

  ~SingleBatteryPublisher() {}

protected:
  void Update() override;
  void Reset() override;
  void PublishBatteryState() override;
  void LogErrors() override;
  void DiagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status) override;
  void DiagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status) override;

private:
  std::shared_ptr<Battery> battery_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_1_pub_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_SINGLE_BATTERY_PUBLISHER_HPP_
