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

#include "panther_battery/roboteq_battery.hpp"

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "panther_utils/moving_average.hpp"

namespace panther_battery
{

RoboteqBattery::RoboteqBattery(
  const std::function<DriverStateMsg::SharedPtr()> & get_driver_state,
  const RoboteqBatteryParams & params)
: GetDriverState(get_driver_state), driver_state_timeout_(params.driver_state_timeout)
{
  voltage_ma_ = std::make_unique<panther_utils::MovingAverage<float>>(
    params.voltage_window_len, std::numeric_limits<float>::quiet_NaN());
  current_ma_ = std::make_unique<panther_utils::MovingAverage<float>>(
    params.current_window_len, std::numeric_limits<float>::quiet_NaN());
}

bool RoboteqBattery::Present() { return true; }

void RoboteqBattery::Update(const rclcpp::Time & header_stamp, const bool /* charger_connected */)
{
  driver_state_ = GetDriverState();
  ValidateDriverStateMsg(header_stamp);

  voltage_raw_ = (driver_state_->front.voltage + driver_state_->rear.voltage) / 2.0f;
  current_raw_ = driver_state_->front.current + driver_state_->rear.current;
  voltage_ma_->Roll(voltage_raw_);
  current_ma_->Roll(current_raw_);

  UpdateBatteryMsgs(header_stamp);
}

void RoboteqBattery::Reset(const rclcpp::Time & header_stamp)
{
  voltage_ma_->Reset();
  current_ma_->Reset();

  ResetBatteryMsgs(header_stamp);
  SetErrorMsg("");
}

void RoboteqBattery::ValidateDriverStateMsg(const rclcpp::Time & header_stamp)
{
  if (!driver_state_) {
    throw std::runtime_error("Waiting for driver state message to arrive");
  }

  rclcpp::Time msg_time(driver_state_->header.stamp);
  if ((header_stamp - msg_time) > rclcpp::Duration::from_seconds(driver_state_timeout_)) {
    throw std::runtime_error("Driver state message timeout");
  }

  if (driver_state_->front.can_net_err || driver_state_->rear.can_net_err) {
    throw std::runtime_error("Motor controller CAN network error");
  }
}

void RoboteqBattery::UpdateBatteryMsgs(const rclcpp::Time & header_stamp)
{
  UpdateBatteryState(header_stamp);
  UpdateBatteryStateRaw();
}

void RoboteqBattery::UpdateBatteryState(const rclcpp::Time & header_stamp)
{
  const float V_bat = voltage_ma_->GetAverage();
  const float I_bat = current_ma_->GetAverage();

  battery_state_.header.stamp = header_stamp;
  battery_state_.voltage = V_bat;
  battery_state_.temperature = std::numeric_limits<float>::quiet_NaN();
  battery_state_.current = I_bat;
  battery_state_.percentage = GetBatteryPercent(V_bat);
  battery_state_.capacity = std::numeric_limits<float>::quiet_NaN();
  battery_state_.design_capacity = kDesignedCapacity;
  battery_state_.charge = battery_state_.percentage * battery_state_.design_capacity;
  battery_state_.cell_voltage = std::vector<float>(
    kNumberOfCells, std::numeric_limits<float>::quiet_NaN());
  battery_state_.cell_temperature = std::vector<float>(
    kNumberOfCells, std::numeric_limits<float>::quiet_NaN());
  battery_state_.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_state_.present = true;
  battery_state_.location = kLocation;
  battery_state_.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;
  battery_state_.power_supply_health = GetBatteryHealth(V_bat);
}

void RoboteqBattery::UpdateBatteryStateRaw()
{
  battery_state_raw_ = battery_state_;
  battery_state_raw_.voltage = voltage_raw_;
  battery_state_raw_.current = current_raw_;
  battery_state_raw_.percentage = GetBatteryPercent(voltage_raw_);
  battery_state_raw_.charge = battery_state_raw_.percentage * battery_state_raw_.design_capacity;
}

std::uint8_t RoboteqBattery::GetBatteryHealth(const float voltage)
{
  if (voltage < kVBatFatalMin) {
    SetErrorMsg("Battery voltage is critically low!");
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
  } else if (voltage > kVBatFatalMax) {
    SetErrorMsg("Battery overvoltage!");
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
  } else {
    SetErrorMsg("");
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
  }
}

}  // namespace panther_battery
