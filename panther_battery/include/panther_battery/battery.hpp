#ifndef PANTHER_BATTERY_BATTERY_HPP_
#define PANTHER_BATTERY_BATTERY_HPP_

#include <algorithm>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

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

  bool HasErrorMsg() const { return !error_msg_.empty(); }

  std::string GetErrorMsg() const { return error_msg_; }
  BatteryStateMsg GetBatteryMsg() const { return battery_state_; }
  BatteryStateMsg GetBatteryMsgRaw() const { return battery_state_raw_; }

protected:
  void SetErrorMsg(const std::string & error_msg) { error_msg_ = error_msg; }

  float GetBatteryPercent(const float voltage) const
  {
    return std::clamp((voltage - V_bat_min_) / (V_bat_full_ - V_bat_min_), 0.0f, 1.0f);
  }

  void ResetBatteryMsgs(const rclcpp::Time & header_stamp)
  {
    battery_state_.header.stamp = header_stamp;
    battery_state_.voltage = std::numeric_limits<float>::quiet_NaN();
    battery_state_.temperature = std::numeric_limits<float>::quiet_NaN();
    battery_state_.current = std::numeric_limits<float>::quiet_NaN();
    battery_state_.percentage = std::numeric_limits<float>::quiet_NaN();
    battery_state_.capacity = std::numeric_limits<float>::quiet_NaN();
    battery_state_.design_capacity = designed_capacity_;
    battery_state_.charge = std::numeric_limits<float>::quiet_NaN();
    battery_state_.cell_voltage =
      std::vector<float>(number_of_cells_, std::numeric_limits<float>::quiet_NaN());
    battery_state_.cell_temperature =
      std::vector<float>(number_of_cells_, std::numeric_limits<float>::quiet_NaN());
    battery_state_.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
    battery_state_.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state_.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
    battery_state_.present = true;
    battery_state_.location = location_;

    battery_state_raw_ = battery_state_;
  }

  static constexpr int number_of_cells_ = 10;
  static constexpr int bat_present_mean_len_ = 10;
  static constexpr float charging_current_thresh_ = 0.1;
  static constexpr float bat_detect_thresh_ = 3.03;
  static constexpr float V_bat_fatal_min_ = 27.0;
  static constexpr float V_bat_fatal_max_ = 43.0;
  static constexpr float V_bat_full_ = 41.4;
  static constexpr float V_bat_min_ = 32.0;
  static constexpr float low_bat_temp_ = -10.0;
  static constexpr float overheat_bat_temp_ = 45.0;
  static constexpr float designed_capacity_ = 20.0;
  static constexpr std::string_view location_ = "user_compartment";

  std::string error_msg_;
  BatteryStateMsg battery_state_;
  BatteryStateMsg battery_state_raw_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_BATTERY_HPP_
