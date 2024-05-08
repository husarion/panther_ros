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

#ifndef PANTHER_BATTERY_BATTERY_HPP_
#define PANTHER_BATTERY_BATTERY_HPP_

#include <algorithm>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

namespace panther_battery
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class Battery
{
public:
  Battery() {}
  ~Battery() {}

  virtual bool Present() = 0;
  virtual void Update(const rclcpp::Time & header_stamp, const bool charger_connected) = 0;
  virtual void Reset(const rclcpp::Time & header_stamp) = 0;

  virtual float GetChargerCurrent() = 0;
  virtual float GetLoadCurrent() = 0;

  bool HasErrorMsg() const { return !error_msg_.empty(); }

  std::string GetErrorMsg() const { return error_msg_; }
  BatteryStateMsg GetBatteryMsg() const { return battery_state_; }
  BatteryStateMsg GetBatteryMsgRaw() const { return battery_state_raw_; }

protected:
  void SetErrorMsg(const std::string & error_msg) { error_msg_ = error_msg; }

  float GetBatteryPercent(const float voltage) const
  {
    return std::clamp((voltage - kVBatMin) / (kVBatFull - kVBatMin), 0.0f, 1.0f);
  }

  void ResetBatteryMsgs(const rclcpp::Time & header_stamp)
  {
    battery_state_.header.stamp = header_stamp;
    battery_state_.voltage = std::numeric_limits<float>::quiet_NaN();
    battery_state_.temperature = std::numeric_limits<float>::quiet_NaN();
    battery_state_.current = std::numeric_limits<float>::quiet_NaN();
    battery_state_.percentage = std::numeric_limits<float>::quiet_NaN();
    battery_state_.capacity = std::numeric_limits<float>::quiet_NaN();
    battery_state_.design_capacity = kDesignedCapacity;
    battery_state_.charge = std::numeric_limits<float>::quiet_NaN();
    battery_state_.cell_voltage = std::vector<float>(
      kNumberOfCells, std::numeric_limits<float>::quiet_NaN());
    battery_state_.cell_temperature = std::vector<float>(
      kNumberOfCells, std::numeric_limits<float>::quiet_NaN());
    battery_state_.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
    battery_state_.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state_.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
    battery_state_.present = true;
    battery_state_.location = kLocation;

    battery_state_raw_ = battery_state_;
  }

  static constexpr int kNumberOfCells = 10;
  static constexpr int kBatPresentMeanLen = 10;
  static constexpr float kChargingCurrentTresh = 0.1;
  static constexpr float kBatDetectTresh = 3.03;
  static constexpr float kVBatFatalMin = 27.0;
  static constexpr float kVBatFatalMax = 43.0;
  static constexpr float kVBatFull = 41.4;
  static constexpr float kVBatMin = 32.0;
  static constexpr float kLowBatTemp = -10.0;
  static constexpr float kOverheatBatTemp = 45.0;
  static constexpr float kDesignedCapacity = 20.0;
  static constexpr std::string_view kLocation = "user_compartment";

  std::string error_msg_;
  BatteryStateMsg battery_state_;
  BatteryStateMsg battery_state_raw_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_BATTERY_HPP_
