#ifndef PANTHER_BATTERY_BATTERY_HPP_
#define PANTHER_BATTERY_BATTERY_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_utils/moving_average.hpp>

namespace panther_battery
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

struct BatteryParams
{
  const float high_bat_temp;
  const float charging_current_tresh;
  const float designed_capacity;
  const int64_t voltage_window_len;
  const int64_t temp_window_len;
  const int64_t current_window_len;
  const int64_t charge_window_len;
};

class Battery
{
public:
  Battery(const BatteryParams & params);

  BatteryStateMsg UpdateBatteryMsg(rclcpp::Time header_stamp);

  BatteryStateMsg UpdateBatteryMsg(
    rclcpp::Time & header_stamp, const float V_bat, const float temp_bat, const float I_bat,
    const float I_charge, const bool charger_connected);

  bool HasErrorMsg() const;
  std::string GetErrorMsg() const;

private:
  static constexpr double V_bat_fatal_min_ = 27.0;
  static constexpr double V_bat_fatal_max_ = 43.0;
  static constexpr double V_bat_full_ = 41.4;
  static constexpr double V_bat_min_ = 32.0;
  static constexpr std::string_view location_ = "user_compartment";

  const float high_bat_temp_;
  const float charging_current_tresh_;
  const float designed_capacity_;
  std::string error_msg_;

  std::unique_ptr<panther_utils::MovingAverage<double>> voltage_ma_;
  std::unique_ptr<panther_utils::MovingAverage<double>> temp_ma_;
  std::unique_ptr<panther_utils::MovingAverage<double>> current_ma_;
  std::unique_ptr<panther_utils::MovingAverage<double>> charge_ma_;
};

inline Battery::Battery(const BatteryParams & params)
: high_bat_temp_(params.high_bat_temp),
  charging_current_tresh_(params.charging_current_tresh),
  designed_capacity_(params.designed_capacity)
{
  voltage_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    params.voltage_window_len, std::numeric_limits<double>::quiet_NaN());
  temp_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    params.temp_window_len, std::numeric_limits<double>::quiet_NaN());
  current_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    params.current_window_len, std::numeric_limits<double>::quiet_NaN());
  charge_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    params.charge_window_len, std::numeric_limits<double>::quiet_NaN());
}

inline BatteryStateMsg Battery::UpdateBatteryMsg(rclcpp::Time header_stamp)
{
  voltage_ma_->Reset();
  temp_ma_->Reset();
  current_ma_->Reset();
  charge_ma_->Reset();

  auto battery_msg = BatteryStateMsg();
  battery_msg.header.stamp = header_stamp;
  battery_msg.voltage = std::numeric_limits<double>::quiet_NaN();
  battery_msg.temperature = std::numeric_limits<double>::quiet_NaN();
  battery_msg.current = std::numeric_limits<double>::quiet_NaN();
  battery_msg.percentage = std::numeric_limits<double>::quiet_NaN();
  battery_msg.capacity = std::numeric_limits<double>::quiet_NaN();
  battery_msg.design_capacity = designed_capacity_;
  battery_msg.charge = std::numeric_limits<double>::quiet_NaN();
  battery_msg.cell_voltage = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  battery_msg.cell_temperature = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_msg.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_msg.present = true;
  battery_msg.location = location_;

  return battery_msg;
}

inline BatteryStateMsg Battery::UpdateBatteryMsg(
  rclcpp::Time & header_stamp, const float V_bat, const float temp_bat, const float I_bat,
  const float I_charge, const bool charger_connected)
{
  voltage_ma_->Roll(V_bat);
  temp_ma_->Roll(temp_bat);
  current_ma_->Roll(I_bat);
  charge_ma_->Roll(I_charge);

  auto V_bat_mean = voltage_ma_->GetAverage();
  auto temp_bat_mean = temp_ma_->GetAverage();
  auto I_bat_mean = current_ma_->GetAverage();
  auto I_charge_mean = charge_ma_->GetAverage();

  auto battery_msg = BatteryStateMsg();
  battery_msg.header.stamp = header_stamp;
  battery_msg.voltage = V_bat_mean;
  battery_msg.temperature = temp_bat_mean;
  battery_msg.current = I_bat_mean;
  battery_msg.percentage =
    std::clamp((V_bat_mean - V_bat_min_) / (V_bat_full_ - V_bat_min_), 0.0, 1.0);
  battery_msg.capacity = std::numeric_limits<double>::quiet_NaN();
  battery_msg.design_capacity = designed_capacity_;
  battery_msg.charge = battery_msg.percentage * battery_msg.design_capacity;
  battery_msg.cell_voltage = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  battery_msg.cell_temperature = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  battery_msg.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_msg.present = true;
  battery_msg.location = location_;

  // check battery status
  if (charger_connected) {
    if (fabs(battery_msg.percentage - 1.0f) < std::numeric_limits<float>::epsilon()) {
      battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_FULL;
    } else if (I_charge_mean > charging_current_tresh_) {
      battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING;
    } else {
      battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
  } else {
    battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;
  }

  // check battery health
  if (V_bat_mean < V_bat_fatal_min_) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
    error_msg_ = "Battery voltage is critically low!";
  } else if (V_bat_mean > V_bat_fatal_max_) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    error_msg_ = "Battery overvoltage!";
  } else if (temp_bat_mean >= high_bat_temp_) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT;
    error_msg_ = "Battery is overheating!";
  } else {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
    error_msg_ = "";
  }

  return battery_msg;
}

inline bool Battery::HasErrorMsg() const
{
  if (error_msg_.empty()) {
    return false;
  }
  return true;
}

inline std::string Battery::GetErrorMsg() const { return error_msg_; }

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_BATTERY_HPP_