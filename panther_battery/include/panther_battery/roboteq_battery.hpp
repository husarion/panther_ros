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

#ifndef PANTHER_BATTERY_ROBOTEQ_BATTERY_HPP_
#define PANTHER_BATTERY_ROBOTEQ_BATTERY_HPP_

#include <cstdint>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "panther_msgs/msg/driver_state.hpp"

#include "panther_battery/battery.hpp"
#include "panther_utils/moving_average.hpp"

namespace panther_battery
{

using DriverStateMsg = panther_msgs::msg::DriverState;

struct RoboteqBatteryParams
{
  const float driver_state_timeout;
  const std::size_t voltage_window_len;
  const std::size_t current_window_len;
};

class RoboteqBattery : public Battery
{
public:
  RoboteqBattery(
    const std::function<DriverStateMsg::SharedPtr()> & get_driver_state,
    const RoboteqBatteryParams & params);

  ~RoboteqBattery() {}

  bool Present() override;
  void Update(const rclcpp::Time & header_stamp, const bool /* charger_connected */) override;
  void Reset(const rclcpp::Time & header_stamp) override;

protected:
  void ValidateDriverStateMsg(const rclcpp::Time & header_stamp);

private:
  void UpdateBatteryMsgs(const rclcpp::Time & header_stamp);
  void UpdateBatteryState(const rclcpp::Time & header_stamp);
  void UpdateBatteryStateRaw();
  std::uint8_t GetBatteryHealth(const float voltage);

  std::function<DriverStateMsg::SharedPtr()> GetDriverState;

  const float driver_state_timeout_;
  float voltage_raw_;
  float current_raw_;
  DriverStateMsg::SharedPtr driver_state_;

  std::unique_ptr<panther_utils::MovingAverage<float>> voltage_ma_;
  std::unique_ptr<panther_utils::MovingAverage<float>> current_ma_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ROBOTEQ_BATTERY_HPP_
